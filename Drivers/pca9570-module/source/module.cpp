// SPDX-License-Identifier: Apache-2.0
#include <tactility/driver.h>
#include <tactility/module.h>

extern "C" {

extern Driver pca9570_driver;

static Driver* const pca9570_drivers[] = {
    &pca9570_driver,
    nullptr
};

Module pca9570_module = {
    .name = "pca9570",
    .start = nullptr,
    .stop = nullptr,
    .drivers = pca9570_drivers,
    .symbols = nullptr,
    .internal = nullptr
};

}
