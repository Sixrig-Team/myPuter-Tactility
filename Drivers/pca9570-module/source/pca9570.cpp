// SPDX-License-Identifier: Apache-2.0
#include <drivers/pca9570.h>
#include <pca9570_module.h>
#include <tactility/device.h>
#include <tactility/driver.h>
#include <tactility/drivers/gpio_controller.h>
#include <tactility/drivers/gpio_descriptor.h>
#include <tactility/drivers/i2c_controller.h>
#include <tactility/log.h>
#include <tactility/concurrent/mutex.h>

#define TAG "PCA9570"

#define GET_CONFIG(device) (static_cast<const Pca9570Config*>((device)->config))

constexpr auto PIN_COUNT = 4;

// Power-on state of the output port: all four outputs high (datasheet, "Power-on reset")
constexpr uint8_t POWER_ON_STATE = 0x0F;

// The chip is output-only, so the output byte is shadowed here instead of being read back from
// the device for every change. Reads are not needed for normal operation at all.
struct Pca9570Internal {
    Mutex mutex {};
    uint8_t output = POWER_ON_STATE;
};

#define GET_INTERNAL(device) (static_cast<Pca9570Internal*>(gpio_controller_get_controller_context(device)))

static error_t start(Device* device) {
    auto* parent = device_get_parent(device);
    if (device_get_type(parent) != &I2C_CONTROLLER_TYPE) {
        LOG_E(TAG, "Parent device is not I2C");
        return ERROR_RESOURCE;
    }

    auto* internal = new Pca9570Internal();
    mutex_construct(&internal->mutex);

    // Pick up the current output state (e.g. after a soft reset), fall back to the power-on state
    uint8_t data;
    error_t error = i2c_controller_read(parent, GET_CONFIG(device)->address, &data, 1, pdMS_TO_TICKS(100));
    if (error == ERROR_NONE) {
        internal->output = data & POWER_ON_STATE;
    } else {
        LOG_W(TAG, "%s: reading output state failed (%s), assuming power-on state", device->name, error_to_string(error));
    }

    LOG_I(TAG, "Started PCA9570 device %s", device->name);
    error = gpio_controller_init_descriptors(device, PIN_COUNT, internal);
    if (error != ERROR_NONE) {
        mutex_destruct(&internal->mutex);
        delete internal;
    }
    return error;
}

static error_t stop(Device* device) {
    auto* internal = GET_INTERNAL(device);
    check(gpio_controller_deinit_descriptors(device) == ERROR_NONE);
    mutex_destruct(&internal->mutex);
    delete internal;
    return ERROR_NONE;
}

extern "C" {

// The PCA9570 has no register address: a write is a single data byte applied directly to
// P3-P0. There is no per-pin command, so every change writes the whole (shadowed) byte.

static error_t set_level(GpioDescriptor* descriptor, bool high) {
    auto* device = descriptor->controller;
    auto* parent = device_get_parent(device);
    auto* internal = GET_INTERNAL(device);
    auto address = GET_CONFIG(device)->address;
    uint8_t bit = static_cast<uint8_t>(1U << descriptor->pin);

    mutex_lock(&internal->mutex);
    uint8_t data = high ? (internal->output | bit) : (internal->output & static_cast<uint8_t>(~bit));
    error_t error = i2c_controller_write(parent, address, &data, 1, portMAX_DELAY);
    if (error == ERROR_NONE) {
        internal->output = data;
    }
    mutex_unlock(&internal->mutex);
    return error;
}

static error_t get_level(GpioDescriptor* descriptor, bool* high) {
    auto* internal = GET_INTERNAL(descriptor->controller);
    mutex_lock(&internal->mutex);
    *high = (internal->output & (1U << descriptor->pin)) != 0;
    mutex_unlock(&internal->mutex);
    return ERROR_NONE;
}

static error_t set_flags(GpioDescriptor* descriptor, gpio_flags_t flags) {
    // The PCA9570 has no input capability: all 4 pins are fixed push-pull outputs.
    if (flags & GPIO_FLAG_DIRECTION_INPUT) {
        LOG_E(TAG, "PCA9570 pins are output-only");
        return ERROR_NOT_SUPPORTED;
    }

    // No pull resistors and no output high-impedance control either.
    if (flags & (GPIO_FLAG_HIGH_IMPEDANCE | GPIO_FLAG_PULL_DOWN)) {
        LOG_E(TAG, "Pull resistors / high impedance are not supported by PCA9570");
        return ERROR_NOT_SUPPORTED;
    }

    return ERROR_NONE;
}

static error_t get_flags(GpioDescriptor* descriptor, gpio_flags_t* flags) {
    *flags = GPIO_FLAG_DIRECTION_OUTPUT;
    return ERROR_NONE;
}

static error_t get_native_pin_number(GpioDescriptor* descriptor, void* pin_number) {
    return ERROR_NOT_SUPPORTED;
}

static error_t add_callback(GpioDescriptor* descriptor, void (*callback)(void*), void* arg) {
    return ERROR_NOT_SUPPORTED;
}

static error_t remove_callback(GpioDescriptor* descriptor) {
    return ERROR_NOT_SUPPORTED;
}

static error_t enable_interrupt(GpioDescriptor* descriptor) {
    return ERROR_NOT_SUPPORTED;
}

static error_t disable_interrupt(GpioDescriptor* descriptor) {
    return ERROR_NOT_SUPPORTED;
}

const static GpioControllerApi pca9570_gpio_api = {
    .set_level = set_level,
    .get_level = get_level,
    .set_flags = set_flags,
    .get_flags = get_flags,
    .get_native_pin_number = get_native_pin_number,
    .add_callback = add_callback,
    .remove_callback = remove_callback,
    .enable_interrupt = enable_interrupt,
    .disable_interrupt = disable_interrupt
};

Driver pca9570_driver = {
    .name = "pca9570",
    .compatible = (const char*[]) { "nxp,pca9570", nullptr},
    .start_device = start,
    .stop_device = stop,
    .api = static_cast<const void*>(&pca9570_gpio_api),
    .device_type = &GPIO_CONTROLLER_TYPE,
    .internal = nullptr
};

}
