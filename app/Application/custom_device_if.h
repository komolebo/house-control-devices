/*
 * dev_if.h
 *
 *  Created on: 17 ���. 2020 �.
 *      Author: Oleh
 */

#ifndef APPLICATION_CUSTOM_DEVICE_IF_H_
#define APPLICATION_CUSTOM_DEVICE_IF_H_

/* include here device headers: */
#include <motion/motion.h>
#include <motion/motion_button_service.h>
#include <motion/motion_data_service.h>
#include <motion/motion_led_service.h>


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
 * FUNCTIONS
 */
void CustomDevice_processGapMessage(uint8_t gap_msg);


#endif /* APPLICATION_CUSTOM_DEVICE_IF_H_ */