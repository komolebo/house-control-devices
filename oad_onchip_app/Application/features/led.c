/*
 * led.c
 *
 *  Created on: 3 трав. 2020 р.
 *      Author: Oleh
 */

#include <bcomdef.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>

#include "device_common.h"
#include "led.h"
#include "util.h"


typedef void (*LedHandlerFunc_t)(ledMode_t reqMode, uint32_t reqBlinkPer,
                                 uint8_t reqBlinkNum);

typedef struct
{
    ledMode_t mode;
    uint32_t blinkPeriod;
    uint8_t blinkNumber;
} ledData_t;

typedef struct
{
    ledMode_t curState;
    ledMode_t transitState;
    LedHandlerFunc_t handler;
} ledSM_t;



ledData_t ledCurrentMode = { .mode = LED_OFF, .blinkNumber = 0,
                             .blinkPeriod = 0 };
ledData_t ledNextMode = { .mode = LED_OFF, .blinkNumber = 0,
                          .blinkPeriod = 0 };


// Clock objects for debouncing the buttons
static Clock_Struct ledClock;
static Clock_Handle ledClockHandle;

/* Pin driver handles */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

static PIN_Config ledPinTable[] =
{
    LED_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

//static uint32_t ledPin;
static uint32_t ledState;

static void Led_clockCallback();

// SM handlers
static void LedSm_switchModeImmediately(ledMode_t reqMode,
                                        uint32_t reqBlinkPer,
                                        uint8_t reqBlinkNum);
static void LedSm_waitUntilCurrentModeDone(ledMode_t reqMode,
                                           uint32_t reqBlinkPer,
                                           uint8_t reqBlinkNum);
static void LedSm_saveCurrModeToRestore(ledMode_t reqMode,
                                        uint32_t reqBlinkPer,
                                        uint8_t reqBlinkNum);
static void LedSm_forceRestartCurrentMode(ledMode_t reqMode,
                                          uint32_t reqBlinkPer,
                                          uint8_t reqBlinkNum);
static LedHandlerFunc_t LedSm_findTransitHandler(ledMode_t from,
                                                 ledMode_t to);
static void Led_processRequest(ledMode_t reqMode, uint32_t reqBlinkPer,
                               uint8_t reqBlinkNum);

static const ledSM_t ledSmTable[] =
{
// { LED_OFF,             LED_OFF,            NULL                          },
 { LED_OFF,             LED_SOLID,          LedSm_switchModeImmediately   },
 { LED_OFF,             LED_BLINK,          LedSm_switchModeImmediately   },
 { LED_OFF,             LED_BLINK_N_TIMES,  LedSm_saveCurrModeToRestore   },

// { LED_SOLID,           LED_SOLID,          NULL                          },
 { LED_SOLID,           LED_OFF,            LedSm_switchModeImmediately   },
 { LED_SOLID,           LED_BLINK,          LedSm_switchModeImmediately   },
 { LED_SOLID,           LED_BLINK_N_TIMES,  LedSm_saveCurrModeToRestore   },

// { LED_BLINK,           LED_BLINK,          NULL                          },
 { LED_BLINK,           LED_OFF,            LedSm_switchModeImmediately   },
 { LED_BLINK,           LED_SOLID,          LedSm_switchModeImmediately   },
 { LED_BLINK,           LED_BLINK_N_TIMES,  LedSm_saveCurrModeToRestore   },

 { LED_BLINK_N_TIMES,   LED_OFF,            LedSm_waitUntilCurrentModeDone },
 { LED_BLINK_N_TIMES,   LED_SOLID,          LedSm_waitUntilCurrentModeDone },
 { LED_BLINK_N_TIMES,   LED_BLINK,          LedSm_waitUntilCurrentModeDone },
 { LED_BLINK_N_TIMES,   LED_BLINK_N_TIMES,  LedSm_forceRestartCurrentMode  }
};


void Led_init(void)
{
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle)
    {
        Log_error0("Error initializing board RLED pins");
        Task_exit();
    }

    ledClockHandle = Util_constructClock(&ledClock,
                                         Led_clockCallback,
                                         0,
                                         0, FALSE, NULL);
    ledCurrentMode.mode = LED_OFF;
}

void Led_on()
{
    Led_processRequest(LED_SOLID, ledCurrentMode.blinkPeriod,
                       ledCurrentMode.blinkNumber);
}

void Led_off()
{
    Led_processRequest(LED_OFF, ledCurrentMode.blinkPeriod,
                       ledCurrentMode.blinkNumber);
}

void Led_blinkNum(uint32_t period, uint8_t num)
{
    Led_processRequest(LED_BLINK_N_TIMES, period, num);
}

void Led_blink(uint32_t period)
{
    Led_processRequest(LED_BLINK, period, ledCurrentMode.blinkNumber);
}

void Led_handleClockEvt()
{
    bool invertLedState = TRUE;

    // invert LED PIN value
    ledState = PIN_getOutputValue(LED_PIN);

    // if LED low: set LED high and start clock
    if ( ledState == LED_LOW )
    {
        Util_startClock((Clock_Struct *)ledClockHandle);
    }
    else // LED high: set LED low and switch to next LED state
    {
        if (ledCurrentMode.mode == LED_BLINK_N_TIMES)
        {
            if (--ledCurrentMode.blinkNumber == 0)
            {
                // restore to next LED mode
                Led_processRequest(ledNextMode.mode, ledNextMode.blinkPeriod,
                                   ledNextMode.blinkNumber);
                invertLedState = FALSE;
            }
            else /* Still have to blink N times */
            {
                Util_startClock((Clock_Struct *)ledClockHandle);
            }
        }
        else /* Periodic blinking */
        {
            Util_startClock((Clock_Struct *)ledClockHandle);
        }
    }

    if (invertLedState)
    {
        PIN_setOutputValue(ledPinHandle, LED_PIN,
                           ledState = (ledState == LED_LOW ? LED_HIGH : LED_LOW));
    }
}

static void LedSm_switchModeImmediately(ledMode_t reqMode,
                                        uint32_t reqBlinkPer,
                                        uint8_t reqBlinkNum)
{
    ledCurrentMode.mode = reqMode;
    ledCurrentMode.blinkPeriod = reqBlinkPer;
    ledCurrentMode.blinkNumber = reqBlinkNum;

    switch (reqMode) {
    case LED_OFF:
    {
        Util_stopClock(&ledClock);
        ledState = 0;
        break;
    }
    case LED_SOLID:
    {
        Util_stopClock(&ledClock);
        ledState = 1;
        break;
    }
    case LED_BLINK:
    {
        ledState = 1;
        Util_restartClock(&ledClock, ledCurrentMode.blinkPeriod);
        break;
    }
    case LED_BLINK_N_TIMES:
        // shouldn't enter here
    default:
        break;
    }

    PIN_setOutputValue(ledPinHandle, LED_PIN, ledState);
}

static void LedSm_waitUntilCurrentModeDone(ledMode_t reqMode,
                                           uint32_t reqBlinkPer,
                                           uint8_t reqBlinkNum)
{
    if (ledCurrentMode.blinkNumber == 0)
    {
        // blinked N times, it's time to switch to the next mode
        ledCurrentMode.mode = reqMode;
        ledCurrentMode.blinkNumber = reqBlinkNum;
        ledCurrentMode.blinkPeriod = reqBlinkPer;

        switch (reqMode) {
        case LED_OFF:
        {
            Util_stopClock(&ledClock);
            ledState = 0;
            break;
        }
        case LED_SOLID:
        {
            Util_stopClock(&ledClock);
            ledState = 1;
            break;
        }
        case LED_BLINK:
        {
            ledState = 1;
            Util_restartClock(&ledClock, ledCurrentMode.blinkPeriod);
            Util_startClock((Clock_Struct *) ledClockHandle);
            break;
        }
        case LED_BLINK_N_TIMES:
            // shouldn't enter here
        default:
            break;
        }

        PIN_setOutputValue(ledPinHandle, LED_PIN, ledState);
    }
    else // busy now, wait until switch is available
    {
        ledNextMode.mode = reqMode;
        ledNextMode.blinkPeriod = reqBlinkPer;
        ledNextMode.blinkNumber = reqBlinkNum;
    }
}

static void LedSm_saveCurrModeToRestore(ledMode_t reqMode,
                                        uint32_t reqBlinkPer,
                                        uint8_t reqBlinkNum)
{
    ledNextMode.mode = ledCurrentMode.mode;
    ledNextMode.blinkPeriod = ledCurrentMode.blinkPeriod;
    ledNextMode.blinkNumber = ledCurrentMode.blinkNumber;

    ledCurrentMode.mode = reqMode;
    ledCurrentMode.blinkPeriod = reqBlinkPer;
    ledCurrentMode.blinkNumber = reqBlinkNum;

    switch (reqMode) {
    case LED_BLINK_N_TIMES:
    {
        ledState = 1;
        Util_restartClock(&ledClock, ledCurrentMode.blinkPeriod);
        PIN_setOutputValue(ledPinHandle, LED_PIN, ledState);
        break;
    }

    case LED_OFF:
    case LED_SOLID:
    case LED_BLINK:
    default:
        // shouldn't enter here
        break;
    }
}

static void LedSm_forceRestartCurrentMode(ledMode_t reqMode,
                                          uint32_t reqBlinkPer,
                                          uint8_t reqBlinkNum)
{
    ledCurrentMode.blinkPeriod = reqBlinkPer;
    ledCurrentMode.blinkNumber = reqBlinkNum;

    switch (reqMode) {
    case LED_BLINK_N_TIMES:
    {
        ledState = 1;
        Util_restartClock(&ledClock, ledCurrentMode.blinkPeriod);
        Util_startClock((Clock_Struct *)ledClockHandle);
        PIN_setOutputValue(ledPinHandle, LED_PIN, ledState);
        break;
    }

    case LED_OFF:
    case LED_SOLID:
    case LED_BLINK:
    default:
        // shouldn't enter here
        break;
    }
}


static LedHandlerFunc_t LedSm_findTransitHandler(ledMode_t from,
                                                 ledMode_t to)
{
    for (uint8_t i = 0; i < sizeof(ledSmTable) / sizeof(ledSmTable[0]); i++)
    {
        if ((ledSmTable[i].curState == from) && (ledSmTable[i].transitState == to))
        {
            return ledSmTable[i].handler;
        }
    }
    return NULL;
}

static void Led_processRequest(ledMode_t reqMode, uint32_t reqBlinkPer,
                               uint8_t reqBlinkNum)
{
    LedHandlerFunc_t handler = LedSm_findTransitHandler(ledCurrentMode.mode,
                                                        reqMode);
    if (handler)
    {
        // Do a transition to a different LED state
        handler(reqMode, reqBlinkPer, reqBlinkNum);
    }
    else
    {
        Log_info0("Led state is already as requested");
    }
}

static void Led_clockCallback()
{
    // Process in application context
    VOID enqueueMsg(EVT_LED_CLK, NULL);
}
