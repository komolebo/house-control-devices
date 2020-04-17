/*
 * dev_if.h
 *
 *  Created on: 17 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_DEV_IF_H_
#define APPLICATION_DEV_IF_H_

/* include here device headers: */
#include "devices/motion.h"

extern const DeviceType DEVICE_TYPE;


void CustomDevice_processGapMessage(uint8_t gap_msg);


#endif /* APPLICATION_DEV_IF_H_ */
