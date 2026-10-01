// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <tactility/bindings/bindings.h>
#include <drivers/pca9570.h>

#ifdef __cplusplus
extern "C" {
#endif

DEFINE_DEVICETREE(pca9570, struct Pca9570Config)

#ifdef __cplusplus
}
#endif
