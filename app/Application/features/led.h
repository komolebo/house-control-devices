/*
 * led.h
 *
 *  Created on: 3 трав. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_FEATURES_LED_H_
#define APPLICATION_FEATURES_LED_H_

typedef enum
{
    LED_OFF,
    LED_SOLID,
    LED_BLINK,
    LED_BLINK_N_TIMES,
    LED_MODE_MAX
} __attribute__ ((packed))  ledMode_t;



void Led_init           (void);

void Led_on             (void);
void Led_off            (void);
void Led_blinkNum       (uint32_t period, uint8_t num);
void Led_blink          (uint32_t period);

void Led_handleClockEvt (void);

#endif /* APPLICATION_FEATURES_LED_H_ */
