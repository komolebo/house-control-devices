/*
 * dev_if.h
 *
 *  Created on: 17 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_CUSTOM_DEVICE_IF_H_
#define APPLICATION_CUSTOM_DEVICE_IF_H_

/* include here device headers: */
#include <motion/motion.h>
#include <services/button_service.h>
#include <services/config_service.h>
#include <services/data_service.h>

#include "device_common.h"


/*********************************************************************
 *  EXTERNAL VARIABLES
 */
extern const uint8_t * SOFTWARE_VERSION;
extern const DeviceType DEVICE_TYPE;

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern void CustomDevice_processGapMessage(uint8_t gap_msg);
extern void CustomDevice_init();

#endif /* APPLICATION_CUSTOM_DEVICE_IF_H_ */
