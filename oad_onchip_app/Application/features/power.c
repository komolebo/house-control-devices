/*
 * power.c
 *
 *  Created on: 16 трав. 2020 р.
 *      Author: Oleh
 */
#include <stdbool.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

#include "Board.h"

#include "power.h"
#include "led.h"


#define PWR_WAKEUP_BLINK_PERIOD_MS          (100)
#define PWR_SHUTDOWN_BLINK_PERIOD_MS        (100)

#define PWR_SHUTDOWN_BLINK_NUM              (4)
#define PWR_WAKEUP_BLINK_NUM                (3)

#define PWR_SHUTDOWN_HOLD_BTN_MS            (3000)

#define PWR_SHUTDOWN_INDICATION_MS          (2 * PWR_SHUTDOWN_BLINK_NUM * PWR_SHUTDOWN_BLINK_PERIOD_MS)



/* Clock used for debounce logic */
Clock_Struct debButtonClock;
Clock_Handle hDebButtonClock;

static Clock_Struct pwrClock;

/* Button pin state */
static PIN_State pwrPinButtonState;

/* Pin driver handles */
PIN_Handle hPwrButton;

/* Wake-up Button pin table */
PIN_Config PowerButtonTable[] = {
    POWER_BTN | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES,
    PIN_TERMINATE                                 /* Terminate list */
};

/* Wake-up Button pin table */
PIN_Config ButtonTableWakeUp[] = {
    POWER_BTN | PIN_INPUT_EN | PIN_PULLUP | PINCC26XX_WAKEUP_NEGEDGE,
    PIN_TERMINATE                                 /* Terminate list */
};


static powerState_t powerState = PWR_NORMAL;


static void Pwr_clockCallback();
static void pwrButtonCb(PIN_Handle handle, PIN_Id pinId);
static void buttonClockCb(UArg arg);


void Pwr_init(void)
{
    Led_init();
    /* Setup button pins with ISR */
    hPwrButton = PIN_open(&pwrPinButtonState, PowerButtonTable);
    if(!hPwrButton)
    {
        Log_error0("Error initializing board Button pins");
        Task_exit();
    }
    PIN_registerIntCb(hPwrButton, pwrButtonCb);

    Clock_Params clockParams;
    Clock_Params_init(&clockParams);
//    clockParams.arg = (UArg)hButtons;
    Clock_construct(&debButtonClock, buttonClockCb, 0, &clockParams);
    hDebButtonClock = Clock_handle(&debButtonClock);


    /*pwrClockHandle = */VOID Util_constructClock(&pwrClock,
                                         Pwr_clockCallback,
                                         0,
                                         0, FALSE, NULL);
}

void Pwr_shutdown(void)
{
    Led_off();

    /* Configure DIO for wake up from shutdown */
    PINCC26XX_setWakeup(ButtonTableWakeUp);

    /* Go to shutdown */
    Power_shutdown(0, 0);
}

void Pwr_wakeup(void)
{
    Led_blinkNum(PWR_WAKEUP_BLINK_PERIOD_MS, PWR_WAKEUP_BLINK_NUM);
}

void Pwr_handleEvt(appEvt_t evt)
{
    switch (evt)
    {
    case EVT_PWR_BTN_PRESS:
    {
        if (powerState == PWR_NORMAL)
        {
            // Starting to shutdown
            Util_restartClock(&pwrClock, PWR_SHUTDOWN_HOLD_BTN_MS);
            powerState = PWR_START_SHUTDOWN;
        }
        break;
    }

    case EVT_PWR_BTN_RELEASE:
    {
        if (powerState == PWR_START_SHUTDOWN)
        {
            if(Util_isActive(&pwrClock))
            {
                // Abort shutdown process
                Util_stopClock(&pwrClock);
                powerState = PWR_NORMAL;
            }
            else
            {
                // Indicate shutdown
                Led_blinkNum(PWR_SHUTDOWN_BLINK_PERIOD_MS,
                             PWR_SHUTDOWN_BLINK_NUM);
                Util_restartClock(&pwrClock,
                                  PWR_SHUTDOWN_INDICATION_MS);
                powerState = PWR_SHUTDOWN_INDICATION;
            }
        }
        break;
    }

    case EVT_PWR_CLK:
    {
        if (powerState == PWR_SHUTDOWN_INDICATION)
        {
            powerState = PWR_SHUTDOWN;
            Pwr_shutdown();
        }
        break;
    }
    default:
        break;
    }
}



/*!*****************************************************************************
 *  @brief      Button clock callback
 *
 *  Called when the debounce periode is over. Stopping the clock, toggling
 *  the device mode based on activeButtonPinId:
 *
 *              Board_PIN_BUTTON1 will put the device in shutdown mode.
 *
 *  Reenabling the interrupts and resetting the activeButtonPinId.
 *
 *  @param      arg  argument (PIN_Handle) connected to the callback
 *
 ******************************************************************************/
static void buttonClockCb(UArg arg)
{
    /* Stop the button clock */
    Clock_stop (hDebButtonClock);

    uint8_t debouncedPinState = PIN_getInputValue(POWER_BTN);

    // Debounce done, handle button events
    if (debouncedPinState == POWER_BUTTON_PRESS_STATE)
    {
        enqueueMsg(EVT_PWR_BTN_PRESS, NULL);
    }
    else if (debouncedPinState == POWER_BUTTON_RELEASE_STATE)
    {
        enqueueMsg(EVT_PWR_BTN_RELEASE, NULL);
    }

    /* Re-enable interrupts to detect button release. */
    PIN_setConfig(hPwrButton, PIN_BM_IRQ, POWER_BTN | PIN_IRQ_BOTHEDGES);
}

/*!*****************************************************************************
 *  @brief      Button callback
 *
 *  Initiates the debounce period by disabling interrupts, setting a timeout
 *  for the button clock callback and starting the button clock.
 *  Sets the activeButtonPinId.
 *
 *  @param      handle PIN_Handle connected to the callback
 *
 *  @param      pinId  PIN_Id of the DIO triggering the callback
 *
 *  @return     none
 ******************************************************************************/
static void pwrButtonCb(PIN_Handle handle, PIN_Id pinId)
{
    /* Disable interrupts during debounce */
    PIN_setConfig(handle, PIN_BM_IRQ, POWER_BTN | PIN_IRQ_DIS);

    /* Set timeout 50 ms from now and re-start clock */
    Clock_setTimeout(hDebButtonClock, (50 * (1000 / Clock_tickPeriod)));
    Clock_start(hDebButtonClock);
}

static void Pwr_clockCallback()
{
    // Process in application context
    VOID enqueueMsg(EVT_PWR_CLK, NULL);
}
