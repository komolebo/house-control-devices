/*
 * motion.c
 *
 *  Created on: 17 квіт. 2020 р.
 *      Author: Oleh
 */
#if MOTION_COMPILE

/*******************************************************************************
 * INCLUDES
 */
#include <motion/motion.h>
#include <device_common.h>
#include <icall.h>
#include <icall_ble_api.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
const DeviceType DEVICE_TYPE = DEVICE_MOTION;
const uint8_t DEVICE_TYPE_NAME[] = "MOTION";

#define REL_VERSION_PATCH   "0"
#define REL_VERSION_MINOR   "1"
#define REL_VERSION_MAJOR   "0"

const uint8_t * SOFTWARE_VERSION =  \
        REL_VERSION_MAJOR "."       \
        REL_VERSION_MINOR "."       \
        REL_VERSION_PATCH;

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */




void CustomDevice_processGapMessage(uint8_t gap_msg);


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

void CustomDevice_processGapMessage(uint8_t gap_msg)
{
    switch (gap_msg)
    {
    default:
        break;
    }
}


#endif
