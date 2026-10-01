# TCA6424A I/O expander

A driver for the `TCA6424A` low-voltage 24-bit I2C-bus/SMBus I/O expander by Texas Instruments.

The 24 pins are exposed as a single GPIO controller with pins 0-23, split internally
over 3 ports of 8 pins each (P0.x = pins 0-7, P1.x = pins 8-15, P2.x = pins 16-23).

Supported per-pin configuration:
- Direction (`GPIO_FLAG_DIRECTION_INPUT` / `GPIO_FLAG_DIRECTION_OUTPUT`)
- Polarity inversion (`GPIO_FLAG_ACTIVE_LOW` / `GPIO_FLAG_ACTIVE_HIGH`)

The chip has no pull-up/pull-down resistors and no output high-impedance control,
so those flags are not supported. Interrupts are not implemented, even though the
chip has a single shared `~INT` pin for all 24 GPIOs.

See https://www.ti.com/product/TCA6424A

License: [Apache v2.0](LICENSE-Apache-2.0.md)
