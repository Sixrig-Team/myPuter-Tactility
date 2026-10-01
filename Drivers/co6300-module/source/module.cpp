// SPDX-License-Identifier: Apache-2.0
#include <tactility/driver.h>
#include <tactility/module.h>

extern "C" {

extern Driver co6300_driver;

static Driver* const co6300_drivers[] = {
    &co6300_driver,
    nullptr
};

Module co6300_module = {
    .name = "co6300",
    .start = nullptr,
    .stop = nullptr,
    .drivers = co6300_drivers,
    .symbols = nullptr,
    .internal = nullptr
};

}
