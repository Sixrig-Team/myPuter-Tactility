// SPDX-License-Identifier: Apache-2.0
#include <tactility/driver.h>
#include <tactility/module.h>

extern "C" {

extern Driver tca6424a_driver;

static Driver* const tca6424a_drivers[] = {
    &tca6424a_driver,
    nullptr
};

Module tca6424a_module = {
    .name = "tca6424a",
    .start = nullptr,
    .stop = nullptr,
    .drivers = tca6424a_drivers,
    .symbols = nullptr,
    .internal = nullptr
};

}
