/*
 * motion.h
 *
 *  Created on: 17 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_MOTION_H_
#define APPLICATION_MOTION_H_
#if MOTION_COMPILE

#include "common_dev.h"



#define ADVERT_DATA_NAME                'M', 'o', 't', 'i', 'o', 'n'
#define ADVERT_DATA_NAME_DISPLAY_LEN    (7)  // and terminal zero '\0'

// Advertisement data
extern uint8_t advertData[ADVERT_DATA_NAME_DISPLAY_LEN + ADVERT_DATA_HEADER_LEN];


#endif
#endif /* APPLICATION_MOTION_H_ */
