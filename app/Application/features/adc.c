/*
 * adc.c
 *
 *  Created on: 4 трав. 2020 р.
 *      Author: Oleh
 */

#include <ti/drivers/ADC.h>
#include <uartlog/UartLog.h>
#include <Board.h>

#include "adc.h"

static ADC_Handle   adcHandle;
static ADC_Params   adcParams;


void Adc_init(void)
{
    ADC_init();
    ADC_Params_init(&adcParams);
    adcHandle = ADC_open(Board_ADC0, &adcParams);

    if (adcHandle == NULL)
    {
        Log_error0("Error initializing ADC0\n");
        while (1);
    }
}

uint32_t Adc_readMedianBySamples(uint8_t samples)
{
    /* Blocking mode conversion */
    int_fast16_t res;
    uint16_t adcValue, minAdcVal = 0xFFFF, maxAdcVal = 0;
    uint32_t mVolt;

    for (uint8_t i = 0; i < samples; i++)
    {
        res = ADC_convert(adcHandle, &adcValue);

        if (res == ADC_STATUS_SUCCESS)
        {
            Log_info2("ADC1 raw result: %d\n, convert result: %d",
                      adcValue, mVolt);
            minAdcVal = MIN(minAdcVal, adcValue);
            maxAdcVal = MAX(maxAdcVal, adcValue);
        }
        else
        {
            Log_error1("ADC1 convert failed (%d)\n", i);
        }
    }

    mVolt = ADC_convertRawToMicroVolts(adcHandle,
                                       (minAdcVal / 2) + (maxAdcVal / 2));

    return mVolt;

}
