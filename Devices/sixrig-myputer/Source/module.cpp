#include <tactility/module.h>

#include <tactility/device.h>
#include <tactility/device_listener.h>
#include <tactility/driver.h>
#include <tactility/drivers/gpio.h>
#include <tactility/drivers/gpio_controller.h>
#include <tactility/drivers/gpio_descriptor.h>
#include <tactility/log.h>

#include <drivers/co6300.h>
#include <drivers/cst3530.h>

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>

constexpr auto* TAG = "MyPuter";

// AUDIO_ENABLE (ESP32-P4 module pad 78) -> load switch U28 -> AUDIO_VDD.
// An unpowered ES8388 clamps the internal I2C bus (i2c0), so this must be on before i2c0 starts.
static constexpr auto AUDIO_ENABLE_PIN = GPIO_NUM_50;

// io_expander0 = TCA6424A (pin numbers are 0-23, port*8 + bit)
constexpr auto TCA_PIN_VCI_EN = 13;      // P15: display VCI power enable
constexpr auto TCA_PIN_LCD_RESET = 15;   // P17: LCD reset
constexpr auto TCA_PIN_TOUCH_RESET = 21; // P25 (TP_RST_1V8): touch controller reset

// lcd_power_expander = PCA9570 (output-only, pins 0-3)
constexpr auto PCA_PIN_LCD_VCC_EN = 3; // P3 (LCD_VCC_EN): main LCD power rail, via load switch U32

// Flattened [cmd, data_len, delay_ms, data...] entries
static const uint8_t DISPLAY_INIT_SEQUENCE[] = {
    0xFE, 1, 0, 0x20,
    0xF4, 1, 0, 0x5A,
    0xF5, 1, 0, 0x59,
    0xFE, 1, 0, 0x80,
    0x03, 1, 0, 0x00,
    0xFE, 1, 0, 0x00,
    0x35, 1, 0, 0x00,
    0x3A, 1, 0, 0x77,
    0x2A, 4, 0, 0x00, 0x00, 0x01, 0x0F,
    0x2B, 4, 0, 0x00, 0x00, 0x03, 0x9F,
    0x53, 1, 0, 0x20,
    0x51, 1, 0, 0xFF,
    0x63, 1, 0, 0xFF,
    0x11, 0, 60,
    0x29, 0, 0,
};

static const Co6300Config display_config = {
    .horizontal_resolution = 272,
    .vertical_resolution = 928,
    .bits_per_pixel = 16,
    .bgr_order = false,
    // LCD_RST is on io_expander0 and is pulsed by power_on_display()
    .pin_reset = GPIO_PIN_SPEC_NONE,
    .ldo_channel = 3,
    .ldo_voltage_mv = 2500,
    .dsi_bus_id = 0,
    .num_data_lanes = 1,
    .lane_bit_rate_mbps = 360,
    .dpi_clock_freq_mhz = 16,
    .hsync_pulse_width = 4,
    .hsync_back_porch = 32,
    .hsync_front_porch = 32,
    .vsync_pulse_width = 4,
    .vsync_back_porch = 8,
    .vsync_front_porch = 8,
    .num_fbs = 1,
    .use_dma2d = true,
    .disable_lp = false,
    // Waiting for scan-out causes stalls/blank flashes when other tasks (e.g. flash writes) block timing
    .allow_tearing = true,
    .init_sequence = DISPLAY_INIT_SEQUENCE,
    .init_sequence_length = sizeof(DISPLAY_INIT_SEQUENCE),
};

static const Cst3530Config touch_config = {
    .address = 0x58,
    .x_max = 272,
    .y_max = 928,
    .swap_xy = false,
    .mirror_x = false,
    .mirror_y = false,
};

static Device display_device {};
static Device touch_device {};

static bool set_output(Device* expander, gpio_pin_t pin, bool level, const char* name) {
    auto* descriptor = gpio_descriptor_acquire(expander, pin, GPIO_FLAG_DIRECTION_OUTPUT, GPIO_OWNER_GPIO);
    if (descriptor == nullptr) {
        LOG_E(TAG, "Failed to acquire %s pin", name);
        return false;
    }
    error_t error = gpio_descriptor_set_level(descriptor, level);
    gpio_descriptor_release(descriptor);
    if (error != ERROR_NONE) {
        LOG_E(TAG, "Failed to set %s: %s", name, error_to_string(error));
        return false;
    }
    return true;
}

// Powers up and resets the display and touch controller.
// When this fails, the panel is unpowered and the CO6300 init would hang in the DSI HAL (it has no timeout).
static bool power_on_display(Device* io_expander0, Device* lcd_power_expander) {
    if (!set_output(lcd_power_expander, PCA_PIN_LCD_VCC_EN, true, "LCD_VCC_EN")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    if (!set_output(io_expander0, TCA_PIN_VCI_EN, true, "VCI_EN")) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    bool ok = set_output(io_expander0, TCA_PIN_LCD_RESET, false, "LCD_RESET") &&
        set_output(io_expander0, TCA_PIN_TOUCH_RESET, false, "TOUCH_RESET");
    vTaskDelay(pdMS_TO_TICKS(10));
    ok = set_output(io_expander0, TCA_PIN_LCD_RESET, true, "LCD_RESET") &&
        set_output(io_expander0, TCA_PIN_TOUCH_RESET, true, "TOUCH_RESET") && ok;
    // The CST3530 needs ~200-300ms after reset before it ACKs on I2C
    vTaskDelay(pdMS_TO_TICKS(300));
    return ok;
}

static bool construct_add_start(Device* device, Device* parent, const char* compatible) {
    error_t error = device_construct(device);
    if (error != ERROR_NONE) {
        LOG_E(TAG, "Failed to construct %s: %s", device->name, error_to_string(error));
        return false;
    }

    // device_set_parent() requires a constructed device, so it can't happen before construct
    device_set_parent(device, parent);

    Driver* driver = driver_find_compatible(compatible);
    if (driver == nullptr) {
        LOG_E(TAG, "No driver registered for %s", compatible);
        device_destruct(device);
        return false;
    }
    device_set_driver(device, driver);

    error = device_add(device);
    if (error != ERROR_NONE) {
        LOG_E(TAG, "Failed to add %s: %s", device->name, error_to_string(error));
        device_destruct(device);
        return false;
    }

    error = device_start(device);
    if (error != ERROR_NONE) {
        LOG_E(TAG, "Failed to start %s: %s", device->name, error_to_string(error));
        device_remove(device);
        device_destruct(device);
        return false;
    }

    return true;
}

static void create_display_and_touch(Device* i2c0) {
    display_device = Device {
        .address = 0,
        .name = "display0",
        .config = &display_config,
        .parent = nullptr,
        .flags = {},
        .internal = nullptr,
    };

    Device* root = nullptr;
    if (device_get_by_name("/", &root) != ERROR_NONE) {
        LOG_E(TAG, "Root device not found");
        return;
    }
    bool started = construct_add_start(&display_device, root, "osptek,co6300");
    device_put(root);
    if (!started) {
        return;
    }

    touch_device = Device {
        .address = 0,
        .name = "touch0",
        .config = &touch_config,
        .parent = nullptr,
        .flags = {},
        .internal = nullptr,
    };
    construct_add_start(&touch_device, i2c0, "hynitron,cst3530");
}

// Fires for every device's start/stop in the system
static void on_device_event(Device* device, DeviceEvent event, void* context) {
    static Device* i2c0 = nullptr;
    static Device* io_expander0 = nullptr;
    static Device* lcd_power_expander = nullptr;
    static bool did_create_display = false;

    if (did_create_display || event != DEVICE_EVENT_STARTED) {
        return;
    }

    if (strcmp(device->name, "i2c0") == 0) {
        i2c0 = device;
    } else if (strcmp(device->name, "io_expander0") == 0) {
        io_expander0 = device;
    } else if (strcmp(device->name, "lcd_power_expander") == 0) {
        lcd_power_expander = device;
    }

    if (i2c0 == nullptr || io_expander0 == nullptr || lcd_power_expander == nullptr) {
        return;
    }
    did_create_display = true;

    if (power_on_display(io_expander0, lcd_power_expander)) {
        create_display_and_touch(i2c0);
    } else {
        LOG_E(TAG, "Display power-up failed, display will not be started");
    }
}

extern "C" {

// This module starts before any devicetree device is constructed,
// so it is the earliest place to power the rails that sit on the I2C bus.
static error_t start() {
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << AUDIO_ENABLE_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    // Set the level before switching to output, so the pin never glitches low
    gpio_set_level(AUDIO_ENABLE_PIN, 1);
    if (gpio_config(&config) != ESP_OK) {
        return ERROR_RESOURCE;
    }
    gpio_set_level(AUDIO_ENABLE_PIN, 1);
    // Let AUDIO_VDD ramp up before the I2C driver starts talking on the bus
    vTaskDelay(pdMS_TO_TICKS(10));

    device_listener_add(on_device_event, nullptr);
    return ERROR_NONE;
}

static error_t stop() {
    device_listener_remove(on_device_event, nullptr);
    return ERROR_NONE;
}

Module sixrig_myputer_module = {
    .name = "sixrig-myputer",
    .start = start,
    .stop = stop,
    .drivers = nullptr,
    .symbols = nullptr,
    .internal = nullptr
};

}
