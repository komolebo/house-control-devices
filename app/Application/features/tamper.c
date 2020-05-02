/*
 * tamper.c
 *
 *  Created on: 2 трав. 2020 р.
 *      Author: Oleh
 */

#include <ti/drivers/PIN.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <uartlog/UartLog.h>

#include "tamper.h"
#include "Board.h"
#include "device_common.h"
#include "util.h"


/* Pin driver handles */
static PIN_Handle tamperPinHandle;
/* Global memory storage for a PIN_Config table */
static PIN_State tamperPinState;
static PIN_Config tamperPinTable[] =
{
    Board_PIN_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

// Clock objects for debouncing the buttons
static Clock_Struct tamperDebounceClock;
static Clock_Handle tamperDebounceClockHandle;

static uint32_t tamperPin;
static uint8_t tamperButtonState = 0;


static void tamperDebounceSwiFxn(UArg buttonId);
static void tamperCallbackFxn(PIN_Handle handle, PIN_Id pinId);



void tamperInit(uint32_t pin)
{
    tamperPin = pin;

    // Open button pins
    tamperPinHandle = PIN_open(&tamperPinState, tamperPinTable);
    if(!tamperPinHandle)
    {
        Log_error0("Error initializing button pins");
        Task_exit();
    }

    // Setup callback for button pins
    if(PIN_registerIntCb(tamperPinHandle, &tamperCallbackFxn) != 0)
    {
        Log_error0("Error registering button callback function");
        Task_exit();
    }

    // Create the debounce clock objects for Button 0 and Button 1
    tamperDebounceClockHandle = Util_constructClock(&tamperDebounceClock,
                                                    tamperDebounceSwiFxn, 50,
                                                    0,
                                                    0,
                                                    pin);
}


/*
 *  Callbacks from Swi-context
 *****************************************************************************/
/*********************************************************************
 * @fn     tamperDebounceSwiFxn
 *
 * @brief  Callback from Clock module on timeout
 *
 *         Determines new state after debouncing
 *
 * @param  buttonId    The pin being debounced
 */
static void tamperDebounceSwiFxn(UArg buttonId)
{
    // Used to send message to app
    ButtonState_t buttonMsg = { .pinId = buttonId };
    uint8_t sendMsg = FALSE;

    // Get current value of the button pin after the clock timeout
    uint8_t buttonPinVal = PIN_getInputValue(buttonId);

    // Set interrupt direction to opposite of debounced state
    // If button is now released (button is active low, so release is high)
    if(buttonPinVal)
    {
        // Enable negative edge interrupts to wait for press
        PIN_setConfig(tamperPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_NEGEDGE);
    }
    else
    {
        // Enable positive edge interrupts to wait for release
        PIN_setConfig(tamperPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_POSEDGE);
    }

    if (buttonId == tamperPin)
    {
        // If button is now released (buttonPinVal is active low, so release is 1)
        // and button state was pressed (buttonstate is active high so press is 1)
        if(buttonPinVal && tamperButtonState)
        {
            // Button was released
            buttonMsg.state = tamperButtonState = 0;
            sendMsg = TRUE;
        }
        else if(!buttonPinVal && !tamperButtonState)
        {
            // Button was pressed
            buttonMsg.state = tamperButtonState = 1;
            sendMsg = TRUE;
        }
    }

    if(sendMsg == TRUE)
    {
        ButtonState_t *pButtonState = ICall_malloc(sizeof(ButtonState_t));
        if(pButtonState != NULL)
        {
            *pButtonState = buttonMsg;
            if(enqueueMsg(TAMPER_DEBOUNCED_EVT, pButtonState) != SUCCESS)
            {
              ICall_free(pButtonState);
            }
        }
    }
}


/*********************************************************************
 * @fn     tamperCallbackFxn
 *
 * @brief  Callback from PIN driver on interrupt
 *
 *         Sets in motion the debouncing.
 *
 * @param  handle    The PIN_Handle instance this is about
 * @param  pinId     The pin that generated the interrupt
 */
static void tamperCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
    Log_info1("Button interrupt: %s",
              (uintptr_t)((pinId == tamperPin) ? "Tamper PIN" : "ERROR"));

    // Disable interrupt on that pin for now. Re-enabled after debounce.
    PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);

    // Start debounce timer
    if(pinId == tamperPin)
    {
        Util_startClock((Clock_Struct *)tamperDebounceClockHandle);
    }
}
