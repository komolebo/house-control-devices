/*
 * power.h
 *
 *  Created on: 16 трав. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_FEATURES_POWER_H_
#define APPLICATION_FEATURES_POWER_H_


#include <ti/sysbios/knl/Semaphore.h>

#include "device_common.h"


typedef enum
{
    PWR_UNDEF,
    PWR_NORMAL,
    PWR_WAKEUP,
    PWR_START_SHUTDOWN,
    PWR_SHUTDOWN_INDICATION,
    PWR_SHUTDOWN,
    PWR_STATE_MAX
} powerState_t;


void Pwr_init(void);
void Pwr_shutdown(void);
void Pwr_wakeup(void);
void Pwr_handleEvt(appEvt_t evt);

#endif /* APPLICATION_FEATURES_POWER_H_ */
