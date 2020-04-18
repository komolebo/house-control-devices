/*
 * common_dev.h
 *
 *  Created on: 17 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_COMMON_DEV_H_
#define APPLICATION_COMMON_DEV_H_

#include <stdint.h>
#include <bcomdef.h>
#include <devinfoservice.h>

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */
#define VAR_NAME_TO_STR(var_name) (#var_name)

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Advert data header len to determine name string address
#define ADVERT_DATA_HEADER_LEN  (5)

/*********************************************************************
 * TYPEDEFS
 */
enum DeviceType
{
    DEVICE_MOTION,
    DEVICE_SMOKE,
    DEVICE_GAS,
    DEVICE_COUNT
};

typedef enum DeviceType DeviceType;
/*********************************************************************
 *  EXTERNAL VARIABLES
 */
extern const uint8_t DevicesName[DEVICE_COUNT][DEVINFO_STR_ATTR_LEN];

/*********************************************************************
 * FUNCTIONS
 */



#endif /* APPLICATION_COMMON_DEV_H_ */
