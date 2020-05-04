/*
 * adc.h
 *
 *  Created on: 4 трав. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_FEATURES_ADC_H_
#define APPLICATION_FEATURES_ADC_H_

#include <icall_ble_api.h>

void Adc_init(void);
uint32_t Adc_readMedianFromSamples(uint8_t samples);

#endif /* APPLICATION_FEATURES_ADC_H_ */
