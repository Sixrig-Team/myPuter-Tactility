# PCA9570 4-bit GPO expander

A driver for the `PCA9570` remote 4-bit general purpose output (GPO) expander by NXP.

The 4 pins (P0-P3) are exposed as a single GPIO controller with pins 0-3. Unlike a regular
GPIO expander, the PCA9570 is **output-only**: it has no input capability and no direction,
polarity, pull-up/down or high-impedance control. `GPIO_FLAG_DIRECTION_INPUT` is therefore not
supported; pins are always in output mode.

The device has no register address byte - a write is a single data byte applied directly to
P3-P0 (P7-P4 are ignored), and a read returns the current output state of P3-P0 (P7-P4 always
read back as 1). It also has no address pins, so it is always found at 7-bit I2C address `0x24` (`0x48` is the 8-bit write address).

The driver keeps a copy of the output byte in RAM, so pin changes are write-only. The device is
read only once at start, to pick up the current state; if that read fails, the power-on state
(all outputs high) is assumed.

See https://www.nxp.com/products/PCA9570

License: [Apache v2.0](LICENSE-Apache-2.0.md)
