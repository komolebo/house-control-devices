/*
 * motion.c
 *
 *  Created on: 17 квіт. 2020 р.
 *      Author: Oleh
 */
#if MOTION_COMPILE

#include "motion.h"
#include "common_dev.h"
#include <icall.h>
#include <icall_ble_api.h>

uint8_t advertData[ADVERT_DATA_NAME_DISPLAY_LEN + ADVERT_DATA_HEADER_LEN] =
{
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // complete name
    ADVERT_DATA_NAME_DISPLAY_LEN, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    ADVERT_DATA_NAME
};


#endif
