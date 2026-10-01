// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct Pca9570Config {
    /** Address on bus (the PCA9570 has no address pins, always 0x24, 7-bit) */
    uint8_t address;
};

#ifdef __cplusplus
}
#endif
