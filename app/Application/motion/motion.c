/*
 * motion.c
 *
 *  Created on: 17 квіт. 2020 р.
 *      Author: Oleh
 */
#if MOTION_COMPILE

/*******************************************************************************
 * INCLUDES
 */
#include <motion/motion.h>
#include <services/button_service.h>
#include <services/config_service.h>
#include <services/data_service.h>
#include <services/tamper_service.h>
#include <device_common.h>
#include <icall.h>
#include <icall_ble_api.h>
#include "features/tamper.h"
#include "features/led.h"
#include "features/adc.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/utils/List.h>
//#include <ti/drivers/ADC.h>

#include <uartlog/UartLog.h>  // Comment out if using xdc Log
#include <ti/display/AnsiColor.h>

#include <string.h>

#include <util.h>
#include <devinfoservice.h>
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
const DeviceType DEVICE_TYPE        =  DEVICE_MOTION;
const uint8_t DEVICE_TYPE_NAME[]    = "MOTION";

#define REL_VERSION_PATCH           "0"
#define REL_VERSION_MINOR           "1"
#define REL_VERSION_MAJOR           "0"

const uint8_t * SOFTWARE_VERSION =  \
        REL_VERSION_MAJOR "."       \
        REL_VERSION_MINOR "."       \
        REL_VERSION_PATCH;

#define MOTION_CALIBRATE_BLINK_PERIOD_MS    (300)

#define MOTION_ADC_MEASURE_SAMPLES          (10)
// Time from data sheet needed to calibrate the device
#define MOTION_CALIBRATION_PERIOD_SEC       (6)

// MCU wakeup period to collect data from sensor
#define MOTION_MEASURE_PERIOD_SEC           (0.5)

// Time that sensor does not generate new signal once motion detected
#define MOTION_DETECTION_HOLDON_SEC         (2)

#define MOTION_DETECTION_THRESHOLD_MV       (2000)

/*********************************************************************
 * TYPEDEFS
 */

typedef void (*motionHandlerFunc_t)(/*ledMode_t reqMode*/);

typedef struct
{
    motionState_t from;
    appEvt_t evt;
    motionState_t to;
    motionHandlerFunc_t func;
} motionSm_t;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
/* Button handling functions */
static void handleButtonPress(ButtonState_t *pState);

/* Profile value change handlers */
static void ProjectZero_updateCharVal(CharacteristicData_t *pCharData);
static void ButtonService_CfgChangeHandler(
    CharacteristicData_t *pCharData);
static void DataService_ValueChangeHandler(
    CharacteristicData_t *pCharData);
static void ConfigService_ValueChangeHandler(
    CharacteristicData_t *pCharData);

static void DataService_ValueChangeCB(uint16_t connHandle,
                                      uint8_t paramID,
                                      uint16_t len,
                                      uint8_t *pValue);
static void ConfigService_ValueChangeCB(uint16_t connHandle,
                                        uint8_t paramID,
                                        uint16_t len,
                                        uint8_t *pValue);
static void DataService_CfgChangeCB(uint16_t connHandle,
                                    uint8_t paramID,
                                    uint16_t len,
                                    uint8_t *pValue);
static void TamperService_CfgChangeCB(uint16_t connHandle,
                                      uint8_t paramID,
                                      uint16_t len,
                                      uint8_t *pValue);
static void MotionSm_handleEvt(appEvt_t evt);
static void MotionSm_handleClockEvt();

static void MotionSm_init(void);
static void MotionSm_disable(void);
static void MotionSm_measure(void);
static void MotionSm_checkPrecond(void);
static void MotionSm_detect(void);
static void MotionSm_checkLedcfg(void);
static bool isLedEnabled();
/*********************************************************************
 * VARIABLES
 */
static motionState_t motionState = MOTION_INIT;
static motionSm_t motionSm[] =
{
 { MOTION_INIT,     EVT_INIT_DONE,      MOTION_CALIBRATE,MotionSm_init         },

 { MOTION_CALIBRATE,EVT_LED_CHANGE,     MOTION_CALIBRATE,MotionSm_checkLedcfg  },
 { MOTION_CALIBRATE,EVT_CHECK_PRECOND,  MOTION_CALIBRATE,MotionSm_checkPrecond },
 { MOTION_CALIBRATE,EVT_DISABLE,        MOTION_DISABLE, MotionSm_disable       },
 { MOTION_CALIBRATE,EVT_MEASURE,        MOTION_MEASURE, MotionSm_measure       },
 { MOTION_MEASURE,  EVT_MEASURE,        MOTION_MEASURE, MotionSm_measure       },

 { MOTION_DISABLE,  EVT_MODE_CHANGE,    MOTION_DISABLE, MotionSm_checkPrecond  },
 { MOTION_DISABLE,  EVT_CONN,           MOTION_DISABLE, MotionSm_checkPrecond  },
 { MOTION_DISABLE,  EVT_MEASURE,        MOTION_MEASURE, MotionSm_measure       },

 { MOTION_MEASURE,  EVT_LED_CHANGE,     MOTION_MEASURE, MotionSm_checkLedcfg   },
 { MOTION_MEASURE,  EVT_MODE_CHANGE,    MOTION_MEASURE, MotionSm_checkPrecond  },
 { MOTION_MEASURE,  EVT_DISCONN,        MOTION_DISABLE, MotionSm_disable       },
 { MOTION_MEASURE,  EVT_DETECT,         MOTION_DETECT,  MotionSm_detect        },
 { MOTION_MEASURE,  EVT_DISABLE,        MOTION_DISABLE, MotionSm_disable       },

 { MOTION_DETECT,   EVT_LED_CHANGE,     MOTION_DETECT,  MotionSm_checkLedcfg   },
 { MOTION_DETECT,   EVT_MODE_CHANGE,    MOTION_DETECT,  MotionSm_checkPrecond  },
 { MOTION_DETECT,   EVT_DISCONN,        MOTION_DISABLE, MotionSm_disable       },
 { MOTION_DETECT,   EVT_MEASURE,        MOTION_MEASURE, MotionSm_measure       },
 { MOTION_DETECT,   EVT_DISABLE,        MOTION_DISABLE, MotionSm_disable       }
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */
// Data Service callback handler.
// The type Data_ServiceCBs_t is defined in data_service.h
static DataServiceCBs_t Data_ServiceCBs =
{
    .pfnChangeCb = DataService_ValueChangeCB,  // Characteristic value change callback handler
    .pfnCfgChangeCb = DataService_CfgChangeCB, // Noti/ind configuration callback handler
};

// Config Service callback handler.
// The type Config_ServiceCBs_t is defined in data_service.h
static ConfigServiceCBs_t Config_ServiceCBs =
{
    .pfnChangeCb = ConfigService_ValueChangeCB,  // Characteristic value change callback handler
    .pfnCfgChangeCb = NULL, // No notification-/indication enabled chars in Config Service
};

// Tamper Service callback handler.
// The type Tamper_ServiceCBs_t is defined in tamper_service.h
static TamperServiceCBs_t Tamper_ServiceCBs =
{
    .pfnChangeCb = NULL,  // No writable chars in Tamper Service, so no change handler.
    .pfnCfgChangeCb = TamperService_CfgChangeCB, // Noti/ind configuration callback handler
};
/*********************************************************************
 * LOCAL VARIABLES
 */
/* Pin driver handles */
static PIN_Handle buttonPinHandle;
/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;

// Clock objects for debouncing the buttons
static Clock_Struct motionClock;

#if 0
static Clock_Handle motionClockHandle;
#endif
/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
static PIN_Config buttonPinTable[] = {
    Board_PIN_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

// Clock objects for debouncing the buttons
static Clock_Struct button1DebounceClock;

/*********************************************************************
 * FUNCTIONS
 */
void CustomDevice_hardwareInit(void)
{
    // ******************************************************************
    // Hardware initialization
    // ******************************************************************
    // Open button pins
    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if(!buttonPinHandle)
    {
        Log_error0("Error initializing button pins");
        Task_exit();
    }


    Adc_init();
    Tamper_init(TAMPER_PIN);
    Led_init();
}

void CustomDevice_bleInit(uint8_t selfEntity)
{
    // ******************************************************************
    // BLE Service initialization
    // ******************************************************************
    DevInfo_AddService(); // Device Information Service

    DevInfo_SetParameter(DEVINFO_SOFTWARE_REV, DEVINFO_STR_ATTR_LEN,
                         (void*)SOFTWARE_VERSION);

    // Add services to GATT server and give ID of this task for Indication acks.
    DataService_AddService(selfEntity);
    ConfigService_AddService(selfEntity);
    TamperService_AddService(selfEntity);

    // Register callbacks with the generated services that
    // can generate events (writes received) to the application
    DataService_RegisterAppCBs(&Data_ServiceCBs);
    ConfigService_RegisterAppCBs(&Config_ServiceCBs);
    TamperService_RegisterAppCBs(&Tamper_ServiceCBs);

#if 0
    // Placeholder variable for characteristic intialization
    uint8_t initVal[40] = {0};
    uint8_t initString[] = "This is a pretty long string, isn't it!";
    // Initalization of characteristics in Data_Service that can provide data.
    DataService_SetParameter(DS_STRING_ID, sizeof(initString), initString);
    DataService_SetParameter(DS_STREAM_ID, DS_STREAM_LEN, initVal);
#endif
}

/*
 *  Callbacks from Swi-context
 *****************************************************************************/
/*
 * @brief  Convenience function for updating characteristic data via pzCharacteristicData_t
 *         structured message.
 *
 * @note   Must run in Task context in case BLE Stack APIs are invoked.
 *
 * @param  *pCharData  Pointer to struct with value to update.
 */
static void ProjectZero_updateCharVal(CharacteristicData_t *pCharData)
{
    switch(pCharData->svcUUID)
    {
        break;
    }
}

/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void ButtonService_CfgChangeHandler(
    CharacteristicData_t *pCharData)
{
    // Cast received data to uint16, as that's the format for CCCD writes.
    uint16_t configValue = *(uint16_t *)pCharData->data;
    char *configValString;

    // Determine what to tell the user
    switch(configValue)
    {
    case GATT_CFG_NO_OPERATION:
        configValString = "Noti/Ind disabled";
        break;
    case GATT_CLIENT_CFG_NOTIFY:
        configValString = "Notifications enabled";
        break;
    case GATT_CLIENT_CFG_INDICATE:
        configValString = "Indications enabled";
        break;
    default:
        configValString = "Unsupported operation";
    }

    switch(pCharData->paramID)
    {
    case BS_BUTTON0_ID:
        Log_info3("CCCD Change msg: %s %s: %s",
                  (uintptr_t)"Button Service",
                  (uintptr_t)"BUTTON0",
                  (uintptr_t)configValString);
        // -------------------------
        // Do something useful with configValue here. It tells you whether someone
        // wants to know the state of this characteristic.
        // ...
        break;

    case BS_BUTTON1_ID:
        Log_info3("CCCD Change msg: %s %s: %s",
                  (uintptr_t)"Button Service",
                  (uintptr_t)"BUTTON1",
                  (uintptr_t)configValString);
        // -------------------------
        // Do something useful with configValue here. It tells you whether someone
        // wants to know the state of this characteristic.
        // ...
        break;
    }
}

/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void DataService_ValueChangeHandler(
    CharacteristicData_t *pCharData)
{
    // Value to hold the received string for printing via Log, as Log printouts
    // happen in the Idle task, and so need to refer to a global/static variable.
#if 0
    static uint8_t received_string[DS_STRING_LEN] = {0};
#endif

    switch(pCharData->paramID)
    {
    case DS_STATE_ID:
    {
        // Do something useful with pCharData->data here
        // -------------------------
        // Copy received data to holder array, ensuring NULL termination.
        uint8_t received_val;
        memset(&received_val, 0, DS_STATE_LEN);
        memcpy(&received_val, pCharData->data, DS_STATE_LEN);
        // Needed to copy before log statement, as the holder array remains after
        // the pCharData message has been freed and reused for something else.
        Log_info3("Value Change msg: %s %s: %d",
                  (uintptr_t)"Data Service",
                  (uintptr_t)"State",
                  received_val);
        break;
    }

#if 0
    case DS_STRING_ID:
        // Do something useful with pCharData->data here
        // -------------------------
        // Copy received data to holder array, ensuring NULL termination.
        memset(received_string, 0, DS_STRING_LEN);
        memcpy(received_string, pCharData->data,
               MIN(pCharData->dataLen, DS_STRING_LEN - 1));
        // Needed to copy before log statement, as the holder array remains after
        // the pCharData message has been freed and reused for something else.
        Log_info3("Value Change msg: %s %s: %s",
                  (uintptr_t)"Data Service",
                  (uintptr_t)"String",
                  (uintptr_t)received_string);
        break;

    case DS_STREAM_ID:
        Log_info3("Value Change msg: Data Service Stream: %02x:%02x:%02x...",
                  pCharData->data[0],
                  pCharData->data[1],
                  pCharData->data[2]);
        // -------------------------
        // Do something useful with pCharData->data here
        break;
#endif
    default:
        return;
    }
}


/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void ConfigService_ValueChangeHandler(
    CharacteristicData_t *pCharData)
{
    // Value to hold the received string for printing via Log, as Log printouts
    // happen in the Idle task, and so need to refer to a global/static variable.
    uint8_t received_val;

    switch(pCharData->paramID)
    {
    case CS_STATE_ID:
        // Do something useful with pCharData->data here
        // -------------------------
        received_val = pCharData->data[0];

        // Needed to copy before log statement, as the holder array remains after
        // the pCharData message has been freed and reused for something else.
        Log_info3("Value Change msg: %s %s: 0x%x",
                  (uintptr_t)"Data Service",
                  (uintptr_t)"State",
                  received_val);
        break;

    case CS_MODE_ID:
        received_val = pCharData->data[0];
        Log_info3("Value Change msg: %s %s: 0x%x",
                  (uintptr_t)"Data Service",
                  (uintptr_t)"Mode",
                  (uint32_t)received_val);
        // -------------------------
        // Do something useful with pCharData->data here
        enqueueMsg(EVT_MODE_CHANGE, NULL);
        break;

    case CS_LED_ID:
    {
        received_val = pCharData->data[0];
        Log_info3("Value Change msg: %s %s: 0x%x",
                  (uintptr_t)"Data Service",
                  (uintptr_t)"Led",
                  (uint32_t)received_val);
        enqueueMsg(EVT_LED_CHANGE, NULL);
        break;
    }
    default:
        return;
    }
}

/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void DataService_CfgChangeHandler(CharacteristicData_t *pCharData)
{
    // Cast received data to uint16, as that's the format for CCCD writes.
    uint16_t configValue = *(uint16_t *)pCharData->data;
    char *configValString;

    // Determine what to tell the user
    switch(configValue)
    {
    case GATT_CFG_NO_OPERATION:
        configValString = "Noti/Ind disabled";
        break;
    case GATT_CLIENT_CFG_NOTIFY:
        configValString = "Notifications enabled";
        break;
    case GATT_CLIENT_CFG_INDICATE:
        configValString = "Indications enabled";
        break;
    default:
        configValString = "Unsupported operation";
    }

    switch(pCharData->paramID)
    {
    case DS_STATE_ID:
            Log_info3("CCCD Change msg: %s %s: %s",
                      (uintptr_t)"Data Service",
                      (uintptr_t)"State",
                      (uintptr_t)configValString);
            // -------------------------
            // Do something useful with configValue here. It tells you whether someone
            // wants to know the state of this characteristic.
            // ...
            break;
#if 0
    case DS_STREAM_ID:
        Log_info3("CCCD Change msg: %s %s: %s",
                  (uintptr_t)"Data Service",
                  (uintptr_t)"Stream",
                  (uintptr_t)configValString);
        // -------------------------
        // Do something useful with configValue here. It tells you whether someone
        // wants to know the state of this characteristic.
        // ...
        break;
#endif
    }
}


/*********************************************************************
 * @fn      DataService_ValueChangeCB
 *
 * @brief   Callback for characteristic change when a peer writes to us
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void DataService_ValueChangeCB(uint16_t connHandle,
                                                  uint8_t paramID, uint16_t len,
                                                  uint8_t *pValue)
{
    // See the service header file to compare paramID with characteristic.
    Log_info1("(CB) Data Svc Characteristic value change: paramID(%d). "
              "Sending msg to app.", paramID);

    CharacteristicData_t *pValChange =
        ICall_malloc(sizeof(CharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = DATA_SERVICE_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        if(enqueueMsg(EVT_SERVICE_WRITE, pValChange) != SUCCESS)
        {
          ICall_free(pValChange);
        }
    }
}


/*********************************************************************
 * @fn      ConfigService_ValueChangeCB
 *
 * @brief   Callback for characteristic change when a peer writes to us
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void ConfigService_ValueChangeCB(uint16_t connHandle,
                                                  uint8_t paramID, uint16_t len,
                                                  uint8_t *pValue)
{
    // See the service header file to compare paramID with characteristic.
    Log_info1("(CB) Config Svc Characteristic value change: paramID(%d). "
              "Sending msg to app.", paramID);

    CharacteristicData_t *pValChange =
        ICall_malloc(sizeof(CharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = CONFIG_SERVICE_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        if(enqueueMsg(EVT_SERVICE_WRITE, pValChange) != SUCCESS)
        {
          ICall_free(pValChange);
        }
    }
}

/*********************************************************************
 * @fn      DataService_CfgChangeCB
 *
 * @brief   Callback for when a peer enables or disables the CCCD attribute,
 *          indicating they are interested in notifications or indications.
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void DataService_CfgChangeCB(uint16_t connHandle,
                                                uint8_t paramID, uint16_t len,
                                                uint8_t *pValue)
{
    Log_info1("(CB) Data Svc Char config change paramID(%d). "
              "Sending msg to app.", paramID);

    CharacteristicData_t *pValChange =
        ICall_malloc(sizeof(CharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = DATA_SERVICE_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        if(enqueueMsg(EVT_SERVICE_CFG, pValChange) != SUCCESS)
        {
          ICall_free(pValChange);
        }
    }
}

void CustomDevice_processGapMessage(uint8_t gap_msg)
{
    switch (gap_msg)
    {
    default:
        break;
    }
}


/*********************************************************************
 * @fn      TamperService_CfgChangeCB
 *
 * @brief   Callback for when a peer enables or disables the CCCD attribute,
 *          indicating they are interested in notifications or indications.
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void TamperService_CfgChangeCB(uint16_t connHandle,
                                      uint8_t paramID, uint16_t len,
                                      uint8_t *pValue)
{
    Log_info1("(CB) Tamper Svc Char config change paramID(%d). "
              "Sending msg to app.", paramID);

    CharacteristicData_t *pValChange =
        ICall_malloc(sizeof(CharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = TAMPER_SERVICE_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        if(enqueueMsg(EVT_SERVICE_CFG, pValChange) != SUCCESS)
        {
          ICall_free(pValChange);
        }
    }
}

/*
 * @brief   Handle a debounced button press or release in Task context.
 *          Invoked by the taskFxn based on a message received from a callback.
 *
 * @see     buttonDebounceSwiFxn
 * @see     buttonCallbackFxn
 *
 * @param   pState  pointer to pzButtonState_t message sent from debounce Swi.
 *
 * @return  None.
 */
static void handleButtonPress(ButtonState_t *pState)
{
    Log_info2("%s %s",
              (uintptr_t)(pState->pinId ==
                      TAMPER_PIN ? "Tamper button" : "Button 1"),
              (uintptr_t)(pState->state ?
                          ANSI_COLOR(FG_GREEN)"pressed"ANSI_COLOR(ATTR_RESET) :
                          ANSI_COLOR(FG_YELLOW)"released"ANSI_COLOR(ATTR_RESET)
                         ));

    // Update the service with the new value.
    // Will automatically send notification/indication if enabled.
    switch(pState->pinId)
    {
    case TAMPER_PIN:
        TamperService_SetParameter(TS_STATE_ID, sizeof(pState->state),
                                   &pState->state);
        break;
    }
}

void CustomDevice_processApplicationMessage(Msg_t *pMsg)
{
    CharacteristicData_t *pCharData = (CharacteristicData_t *)pMsg->pData;

    switch(pMsg->event)
    {
        case EVT_UPDATE_CHARVAL: /* Message from ourselves to send  */
            ProjectZero_updateCharVal(pCharData);
            break;

        case EVT_BUTTON_DEBOUNCED: /* Message from swi about pin change */
        {
            ButtonState_t *pButtonState = (ButtonState_t *)pMsg->pData;
            handleButtonPress(pButtonState);
            break;
        }

        case EVT_TAMPER_CHANGED: /* Message from swi about pin change */
        {
            ButtonState_t *pButtonState = (ButtonState_t *)pMsg->pData;
            handleButtonPress(pButtonState);
            break;
        }

        case EVT_LED_CLK:
        {
            Led_handleClockEvt();
            break;
        }

        case EVT_MOTION_CLK:
        {
            MotionSm_handleClockEvt();
            break;
        }

        case EVT_SERVICE_CFG: /* Message about received CCCD write */
            /* Call different handler per service */
            switch(pCharData->svcUUID)
            {
              case BUTTON_SERVICE_SERV_UUID:
                  ButtonService_CfgChangeHandler(pCharData);
                  break;
              case DATA_SERVICE_SERV_UUID:
                  DataService_CfgChangeHandler(pCharData);
                  break;
            }
            break;

        case EVT_SERVICE_WRITE: /* Message about received value write */
            /* Call different handler per service */
            switch(pCharData->svcUUID)
            {
              case DATA_SERVICE_SERV_UUID:
                  DataService_ValueChangeHandler(pCharData);
                  break;
              case CONFIG_SERVICE_SERV_UUID:
                  ConfigService_ValueChangeHandler(pCharData);
            }
            break;
    }

    MotionSm_handleEvt((appEvt_t)pMsg->event);
}

static void MotionSm_handleEvt(appEvt_t evt)
{
    motionHandlerFunc_t handler;
    for(uint8_t i = 0; i < sizeof(motionSm) / sizeof(motionSm[0]); i++)
    {
        if(motionSm[i].evt == evt && motionSm[i].from == motionState)
        {
            if ((handler = motionSm[i].func) != NULL)
            {
                Log_info3("%s: transition %d -> %d", (uintptr_t)__func__,
                          motionState,
                          motionSm[i].to);
                handler();
                motionState = motionSm[i].to;
            }
        }
    }
}

static void Motion_swiFxn(UArg arg)
{
    VOID arg;
    VOID enqueueMsg(EVT_MOTION_CLK, NULL);
}

static void MotionSm_handleClockEvt()
{
    Log_info0("Motion timer expired");
    switch (motionState) {
        case MOTION_CALIBRATE:
        { /* Calibration finished */
            Util_stopClock(&motionClock);
            Led_off();

            // check connection and configuration
            enqueueMsg(EVT_CHECK_PRECOND, NULL) ;
            break;
        }

        case MOTION_MEASURE:
        {
            Util_restartClock(&motionClock,
                              1000 * MOTION_MEASURE_PERIOD_SEC);
            enqueueMsg(EVT_MEASURE, NULL);
            break;
        }

        case MOTION_DETECT:
        {
            enqueueMsg(EVT_MEASURE, NULL);
            break;
        }

        default:
            break;
    }
}

static void MotionSm_init(void)
{
    Log_info1("%s", (uintptr_t)__func__);
    // TODO: optimize clock handles
    /*motionClockHandle = */
    VOID Util_constructClock(&motionClock,
                            Motion_swiFxn,
                            1000 * MOTION_CALIBRATION_PERIOD_SEC,
                            0, 0, 0);

    // start calibration here
    Util_restartClock(&motionClock, 1000 * MOTION_CALIBRATION_PERIOD_SEC);
    if (isLedEnabled())
    {
        Led_blink(MOTION_CALIBRATE_BLINK_PERIOD_MS);
    }
}

static void MotionSm_checkPrecond(void)
{
    Log_info1("%s", (uintptr_t)__func__);
    // check to enable/disable the feature if it's disabled/enabled respectively
    uint8_t connected = 1, configEnabled;
    uint16_t readLen = 1;

    connected = isConnected();
    ConfigService_GetParameter(CS_MODE_ID, &readLen, &configEnabled);

    Log_info4("%s: connected: %d, config enabled: %d, led enabled: %d",
              (uintptr_t )__func__, connected, configEnabled, isLedEnabled());

    enqueueMsg((connected && configEnabled) ? EVT_MEASURE : EVT_DISABLE, NULL);

    if (isLedEnabled())
    {
        Led_off();
    }
}

static void MotionSm_disable(void)
{
    Log_info1("%s", (uintptr_t)__func__);

    // stop everything
    Util_stopClock(&motionClock);
    Led_off();
}

static void MotionSm_measure(void)
{
    Log_info1("%s", (uintptr_t)__func__);
    // measure and start clock
//#if 0
    uint32_t microVolt = Adc_readMedianFromSamples(MOTION_ADC_MEASURE_SAMPLES);
    if (microVolt >= MOTION_DETECTION_THRESHOLD_MV)
    {
        Util_stopClock(&motionClock);
        enqueueMsg(EVT_DETECT, NULL);
    }
    else
    { // motion not detected, keep measuring
        // TODO: set correct service
        uint8_t val = 0;
        DataService_SetParameter(DS_STATE_ID, 1, &val);
        Util_startClock(&motionClock);
        Led_off();
    }
//#endif
}

static void MotionSm_detect(void)
{
    Log_info1("%s", (uintptr_t)__func__);

    // lid the LED and start detection clock, send notification
    if(isLedEnabled())
    {
        Led_on();
    }
    Util_restartClock(&motionClock, MOTION_DETECTION_HOLDON_SEC * 1000);

    // TODO: set correct service
    uint8_t val = DS_TRIGGERED;
    DataService_SetParameter(DS_STATE_ID, 1, &val);
}

static bool isLedEnabled()
{
    uint16_t readLen;
    uint8_t ledEnabled;

    ConfigService_GetParameter(CS_LED_ID, &readLen, &ledEnabled);

    return ledEnabled;
}

static void MotionSm_checkLedcfg(void)
{
    // turn off if LED is disabled by BACK-END
    if (isLedEnabled() == false)
    {
        Led_off();
    }
}

#endif
