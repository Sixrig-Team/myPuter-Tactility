// SPDX-License-Identifier: Apache-2.0
#include <tactility/driver.h>
#include <tactility/module.h>

extern "C" {

extern Driver cst3530_driver;

static Driver* const cst3530_drivers[] = {
    &cst3530_driver,
    nullptr
};

Module cst3530_module = {
    .name = "cst3530",
    .start = nullptr,
    .stop = nullptr,
    .drivers = cst3530_drivers,
    .symbols = nullptr,
    .internal = nullptr
};

} // extern "C"
