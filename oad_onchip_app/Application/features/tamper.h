/*
 * tamper.h
 *
 *  Created on: 2 трав. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_FEATURES_TAMPER_H_
#define APPLICATION_FEATURES_TAMPER_H_

#include <ti/drivers/PIN.h>


typedef struct
{
    PIN_Id pinId;
    uint8_t state;
} ButtonState_t;

void Tamper_init(uint32_t pin);




#endif /* APPLICATION_FEATURES_TAMPER_H_ */
