/*
 * dev_if.h
 *
 *  Created on: 17 ���. 2020 �.
 *      Author: Oleh
 */

#ifndef APPLICATION_DEV_IF_H_
#define APPLICATION_DEV_IF_H_

/* include here device headers: */
#include "devices/motion.h"


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


#endif /* APPLICATION_DEV_IF_H_ */
