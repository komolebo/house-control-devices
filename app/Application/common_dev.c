/*
 * common_dev.c
 *
 *  Created on: 18 ���. 2020 �.
 *      Author: Oleh
 */

#include "common_dev.h"

const uint8_t * DevicesName[DEVICE_COUNT] =
{
    [DEVICE_MOTION]     = "MOTION",
    [DEVICE_SMOKE]      = "SMOKE",
    [DEVICE_GAS]        = "GAS",
};
