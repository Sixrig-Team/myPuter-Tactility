// SPDX-License-Identifier: Apache-2.0
#include <drivers/cst3530.h>

#include <cst3530_module.h>

#include <tactility/check.h>
#include <tactility/delay.h>
#include <tactility/device.h>
#include <tactility/driver.h>
#include <tactility/drivers/i2c_controller.h>
#include <tactility/drivers/pointer.h>
#include <tactility/error.h>
#include <tactility/log.h>

#include <cstdlib>
#include <utility>

#define TAG "CST3530"
#define GET_CONFIG(device) (static_cast<const Cst3530Config*>((device)->config))

static const TickType_t I2C_TIMEOUT = pdMS_TO_TICKS(100);

// Registers are addressed with a 4-byte big-endian command.
// Protocol ported from the vendor driver at https://github.com/osptek-tech/3.19-amoled-272x928-mipi-co6300
static constexpr uint32_t REG_REPORT = 0xD0070000;
static constexpr uint32_t CMD_CLEAR_REPORT = 0xD00002AB;
static constexpr uint8_t REPORT_TYPE_TOUCH = 0xFF;
static constexpr uint8_t CST3530_MAX_POINTS = 5;
static constexpr size_t REPORT_SIZE = 64;

struct Cst3530Internal {
    bool swap_xy;
    bool mirror_x;
    bool mirror_y;
    uint8_t point_count;
    uint16_t x[CST3530_MAX_POINTS];
    uint16_t y[CST3530_MAX_POINTS];
    uint8_t strength[CST3530_MAX_POINTS];
};

// The controller needs a short pause after each command before it accepts the next transfer
static error_t write_command(Device* i2c, uint8_t address, uint32_t command) {
    const uint8_t bytes[4] = {
        static_cast<uint8_t>(command >> 24),
        static_cast<uint8_t>(command >> 16),
        static_cast<uint8_t>(command >> 8),
        static_cast<uint8_t>(command)
    };
    error_t error = i2c_controller_write(i2c, address, bytes, sizeof(bytes), I2C_TIMEOUT);
    delay_micros(100);
    return error;
}

static error_t read_register(Device* i2c, uint8_t address, uint32_t reg, uint8_t* data, size_t data_size) {
    error_t error = write_command(i2c, address, reg);
    if (error != ERROR_NONE) {
        return error;
    }
    return i2c_controller_read(i2c, address, data, data_size, I2C_TIMEOUT);
}

// region Driver lifecycle

static error_t start(Device* device) {
    auto* parent = device_get_parent(device);
    check(device_get_type(parent) == &I2C_CONTROLLER_TYPE);

    const auto* config = GET_CONFIG(device);

    auto* internal = static_cast<Cst3530Internal*>(calloc(1, sizeof(Cst3530Internal)));
    if (internal == nullptr) {
        return ERROR_OUT_OF_MEMORY;
    }
    internal->swap_xy = config->swap_xy;
    internal->mirror_x = config->mirror_x;
    internal->mirror_y = config->mirror_y;

    // There is no known ID register, so probe by reading the report.
    // Not fatal: the controller may still be booting.
    uint8_t buffer[4];
    error_t error = read_register(parent, config->address, REG_REPORT, buffer, sizeof(buffer));
    if (error != ERROR_NONE) {
        LOG_W(TAG, "Probe at 0x%02X failed: %s (continuing)", config->address, error_to_string(error));
    }

    device_set_driver_data(device, internal);
    return ERROR_NONE;
}

static error_t stop(Device* device) {
    auto* internal = static_cast<Cst3530Internal*>(device_get_driver_data(device));
    free(internal);
    device_set_driver_data(device, nullptr);
    return ERROR_NONE;
}

// endregion

// region PointerApi

static error_t cst3530_read_data(Device* device, TickType_t /*timeout*/) {
    auto* internal = static_cast<Cst3530Internal*>(device_get_driver_data(device));
    const auto* config = GET_CONFIG(device);
    auto* parent = device_get_parent(device);

    internal->point_count = 0;

    uint8_t buffer[REPORT_SIZE] = {};
    if (read_register(parent, config->address, REG_REPORT, buffer, sizeof(buffer)) != ERROR_NONE) {
        return ERROR_RESOURCE;
    }

    write_command(parent, config->address, CMD_CLEAR_REPORT);

    const uint8_t report_type = buffer[2];
    uint8_t finger_count = buffer[3] & 0x0F;
    const uint8_t key_count = buffer[3] >> 4;
    if (report_type != REPORT_TYPE_TOUCH || finger_count == 0) {
        return ERROR_NONE;
    }
    if (finger_count > CST3530_MAX_POINTS) {
        finger_count = CST3530_MAX_POINTS;
    }

    // Each point is 5 bytes, starting at offset 4 and preceded by the key slots
    uint8_t valid_points = 0;
    for (uint8_t i = 0; i < finger_count; i++) {
        const size_t index = (key_count + i) * 5;
        if (index + 7 >= REPORT_SIZE) {
            break;
        }

        uint16_t x = buffer[index + 4] | static_cast<uint16_t>((buffer[index + 7] & 0x0F) << 8);
        uint16_t y = buffer[index + 5] | static_cast<uint16_t>((buffer[index + 7] & 0xF0) << 4);
        const uint8_t strength = buffer[index + 6];
        if (x >= config->x_max || y >= config->y_max || strength == 0) {
            continue;
        }

        if (internal->mirror_x) {
            x = config->x_max - 1 - x;
        }
        if (internal->mirror_y) {
            y = config->y_max - 1 - y;
        }
        if (internal->swap_xy) {
            std::swap(x, y);
        }

        internal->x[valid_points] = x;
        internal->y[valid_points] = y;
        internal->strength[valid_points] = strength;
        valid_points++;
    }
    internal->point_count = valid_points;
    return ERROR_NONE;
}

static bool cst3530_get_touched_points(Device* device, uint16_t* x, uint16_t* y, uint16_t* strength, uint8_t* point_count, uint8_t max_point_count) {
    auto* internal = static_cast<Cst3530Internal*>(device_get_driver_data(device));
    uint8_t count = internal->point_count < max_point_count ? internal->point_count : max_point_count;
    for (uint8_t i = 0; i < count; i++) {
        x[i] = internal->x[i];
        y[i] = internal->y[i];
        if (strength != nullptr) {
            strength[i] = internal->strength[i];
        }
    }
    *point_count = count;
    return count > 0;
}

static error_t cst3530_set_swap_xy(Device* device, bool swap) {
    static_cast<Cst3530Internal*>(device_get_driver_data(device))->swap_xy = swap;
    return ERROR_NONE;
}

static error_t cst3530_get_swap_xy(Device* device, bool* swap) {
    *swap = static_cast<Cst3530Internal*>(device_get_driver_data(device))->swap_xy;
    return ERROR_NONE;
}

static error_t cst3530_set_mirror_x(Device* device, bool mirror) {
    static_cast<Cst3530Internal*>(device_get_driver_data(device))->mirror_x = mirror;
    return ERROR_NONE;
}

static error_t cst3530_get_mirror_x(Device* device, bool* mirror) {
    *mirror = static_cast<Cst3530Internal*>(device_get_driver_data(device))->mirror_x;
    return ERROR_NONE;
}

static error_t cst3530_set_mirror_y(Device* device, bool mirror) {
    static_cast<Cst3530Internal*>(device_get_driver_data(device))->mirror_y = mirror;
    return ERROR_NONE;
}

static error_t cst3530_get_mirror_y(Device* device, bool* mirror) {
    *mirror = static_cast<Cst3530Internal*>(device_get_driver_data(device))->mirror_y;
    return ERROR_NONE;
}

// endregion

static const PointerApi cst3530_pointer_api = {
    .enter_sleep = nullptr,
    .exit_sleep = nullptr,
    .read_data = cst3530_read_data,
    .get_touched_points = cst3530_get_touched_points,
    .set_swap_xy = cst3530_set_swap_xy,
    .get_swap_xy = cst3530_get_swap_xy,
    .set_mirror_x = cst3530_set_mirror_x,
    .get_mirror_x = cst3530_get_mirror_x,
    .set_mirror_y = cst3530_set_mirror_y,
    .get_mirror_y = cst3530_get_mirror_y,
};

Driver cst3530_driver = {
    .name = "cst3530",
    .compatible = (const char*[]) { "hynitron,cst3530", nullptr },
    .start_device = start,
    .stop_device = stop,
    .api = &cst3530_pointer_api,
    .device_type = &POINTER_TYPE,
    .owner = &cst3530_module,
    .internal = nullptr
};
