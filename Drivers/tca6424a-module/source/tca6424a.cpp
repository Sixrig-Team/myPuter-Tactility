// SPDX-License-Identifier: Apache-2.0
#include <drivers/tca6424a.h>
#include <tca6424a_module.h>
#include <tactility/device.h>
#include <tactility/driver.h>
#include <tactility/drivers/gpio_controller.h>
#include <tactility/drivers/gpio_descriptor.h>
#include <tactility/drivers/i2c_controller.h>
#include <tactility/log.h>

#define TAG "TCA6424A"

#define GET_CONFIG(device) (static_cast<const Tca6424aConfig*>((device)->config))

constexpr auto PIN_COUNT = 24;

// The 24 pins are organized as 3 ports of 8 pins each (P0.x, P1.x, P2.x).
// Each register type occupies one command byte per port, at a fixed offset from its base.
constexpr auto TCA6424A_REGISTER_INPUT_BASE = 0x00;
constexpr auto TCA6424A_REGISTER_OUTPUT_BASE = 0x04;
constexpr auto TCA6424A_REGISTER_POLARITY_INVERSION_BASE = 0x08;
constexpr auto TCA6424A_REGISTER_CONFIGURATION_BASE = 0x0C;

static inline uint8_t port_of(gpio_pin_t pin) {
    return pin >> 3;
}

static inline uint8_t bit_of(gpio_pin_t pin) {
    return static_cast<uint8_t>(1U << (pin & 0x07));
}

static error_t start(Device* device) {
    auto* parent = device_get_parent(device);
    if (device_get_type(parent) != &I2C_CONTROLLER_TYPE) {
        LOG_E(TAG, "Parent device is not I2C");
        return ERROR_RESOURCE;
    }
    LOG_I(TAG, "Started TCA6424A device %s", device->name);

    return gpio_controller_init_descriptors(device, PIN_COUNT, nullptr);
}

static error_t stop(Device* device) {
    check(gpio_controller_deinit_descriptors(device) == ERROR_NONE);
    return ERROR_NONE;
}

extern "C" {

static error_t set_level(GpioDescriptor* descriptor, bool high) {
    auto* device = descriptor->controller;
    auto* parent = device_get_parent(device);
    auto address = GET_CONFIG(device)->address;
    auto reg = static_cast<uint8_t>(TCA6424A_REGISTER_OUTPUT_BASE + port_of(descriptor->pin));
    uint8_t bit = bit_of(descriptor->pin);

    if (high) {
        return i2c_controller_register8_set_bits(parent, address, reg, bit, portMAX_DELAY);
    } else {
        return i2c_controller_register8_reset_bits(parent, address, reg, bit, portMAX_DELAY);
    }
}

static error_t get_level(GpioDescriptor* descriptor, bool* high) {
    auto* device = descriptor->controller;
    auto* parent = device_get_parent(device);
    auto address = GET_CONFIG(device)->address;
    auto reg = static_cast<uint8_t>(TCA6424A_REGISTER_INPUT_BASE + port_of(descriptor->pin));
    uint8_t bits;

    error_t err = i2c_controller_register8_get(parent, address, reg, &bits, portMAX_DELAY);
    if (err != ERROR_NONE) {
        return err;
    }

    *high = (bits & bit_of(descriptor->pin)) != 0;
    return ERROR_NONE;
}

static error_t set_flags(GpioDescriptor* descriptor, gpio_flags_t flags) {
    // The TCA6424A has no pull-up/pull-down resistors and no output high-impedance control.
    if (flags & GPIO_FLAG_HIGH_IMPEDANCE) {
        LOG_E(TAG, "High impedance is not supported by TCA6424A");
        return ERROR_NOT_SUPPORTED;
    }

    auto* device = descriptor->controller;
    auto* parent = device_get_parent(device);
    auto address = GET_CONFIG(device)->address;
    uint8_t bit = bit_of(descriptor->pin);
    auto configuration_reg = static_cast<uint8_t>(TCA6424A_REGISTER_CONFIGURATION_BASE + port_of(descriptor->pin));
    auto polarity_reg = static_cast<uint8_t>(TCA6424A_REGISTER_POLARITY_INVERSION_BASE + port_of(descriptor->pin));

    error_t err;

    // Direction: on TCA6424A, a set configuration bit means input, a cleared bit means output.
    if (flags & GPIO_FLAG_DIRECTION_OUTPUT) {
        err = i2c_controller_register8_reset_bits(parent, address, configuration_reg, bit, portMAX_DELAY);
    } else {
        err = i2c_controller_register8_set_bits(parent, address, configuration_reg, bit, portMAX_DELAY);
    }

    if (err != ERROR_NONE) {
        return err;
    }

    // Polarity inversion
    if (flags & GPIO_FLAG_ACTIVE_LOW) {
        err = i2c_controller_register8_set_bits(parent, address, polarity_reg, bit, portMAX_DELAY);
    } else {
        err = i2c_controller_register8_reset_bits(parent, address, polarity_reg, bit, portMAX_DELAY);
    }

    return err;
}

static error_t get_flags(GpioDescriptor* descriptor, gpio_flags_t* flags) {
    auto* device = descriptor->controller;
    auto* parent = device_get_parent(device);
    auto address = GET_CONFIG(device)->address;
    uint8_t bit = bit_of(descriptor->pin);
    auto configuration_reg = static_cast<uint8_t>(TCA6424A_REGISTER_CONFIGURATION_BASE + port_of(descriptor->pin));
    auto polarity_reg = static_cast<uint8_t>(TCA6424A_REGISTER_POLARITY_INVERSION_BASE + port_of(descriptor->pin));
    uint8_t val;
    error_t err;

    gpio_flags_t f = GPIO_FLAG_NONE;

    // Direction
    err = i2c_controller_register8_get(parent, address, configuration_reg, &val, portMAX_DELAY);
    if (err != ERROR_NONE) return err;
    if (val & bit) {
        f |= GPIO_FLAG_DIRECTION_INPUT;
    } else {
        f |= GPIO_FLAG_DIRECTION_OUTPUT;
    }

    // Polarity inversion
    err = i2c_controller_register8_get(parent, address, polarity_reg, &val, portMAX_DELAY);
    if (err != ERROR_NONE) return err;
    if (val & bit) {
        f |= GPIO_FLAG_ACTIVE_LOW;
    }

    *flags = f;
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

const static GpioControllerApi tca6424a_gpio_api = {
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

Driver tca6424a_driver = {
    .name = "tca6424a",
    .compatible = (const char*[]) { "ti,tca6424a", nullptr},
    .start_device = start,
    .stop_device = stop,
    .api = static_cast<const void*>(&tca6424a_gpio_api),
    .device_type = &GPIO_CONTROLLER_TYPE,
    .owner = &tca6424a_module,
    .internal = nullptr
};

}
