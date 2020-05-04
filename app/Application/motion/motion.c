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
#include <devinfoservice.h>
#include <device_common.h>
#include <icall.h>
#include <icall_ble_api.h>
#include "features/tamper.h"
#include "features/led.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/utils/List.h>

#include <uartlog/UartLog.h>  // Comment out if using xdc Log
#include <ti/display/AnsiColor.h>

#include <string.h>

#include <util.h>
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

/*********************************************************************
 * TYPEDEFS
 */


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

/*********************************************************************
 * VARIABLES
 */


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
/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
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

    // Placeholder variable for characteristic intialization
    uint8_t initVal[40] = {0};
    uint8_t initString[] = "This is a pretty long string, isn't it!";

    // Initalization of characteristics in Data_Service that can provide data.
    DataService_SetParameter(DS_STRING_ID, sizeof(initString), initString);
    DataService_SetParameter(DS_STREAM_ID, DS_STREAM_LEN, initVal);
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
    case BUTTON_SERVICE_SERV_UUID:
        ButtonService_SetParameter(pCharData->paramID, pCharData->dataLen,
                                   pCharData->data);
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
    static uint8_t received_string[DS_STRING_LEN] = {0};

    switch(pCharData->paramID)
    {
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
        break;

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

        if(enqueueMsg(PZ_SERVICE_WRITE_EVT, pValChange) != SUCCESS)
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

        if(enqueueMsg(PZ_SERVICE_WRITE_EVT, pValChange) != SUCCESS)
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

        if(enqueueMsg(PZ_SERVICE_CFG_EVT, pValChange) != SUCCESS)
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

        if(enqueueMsg(PZ_SERVICE_CFG_EVT, pValChange) != SUCCESS)
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
        case PZ_UPDATE_CHARVAL_EVT: /* Message from ourselves to send  */
            ProjectZero_updateCharVal(pCharData);
            break;

        case PZ_BUTTON_DEBOUNCED_EVT: /* Message from swi about pin change */
        {
            ButtonState_t *pButtonState = (ButtonState_t *)pMsg->pData;
            handleButtonPress(pButtonState);
            break;
        }

        case TAMPER_DEBOUNCED_EVT: /* Message from swi about pin change */
        {
            ButtonState_t *pButtonState = (ButtonState_t *)pMsg->pData;
            handleButtonPress(pButtonState);
            break;
        }

        case LED_CLK_EVT:
        {
            Led_handleClockEvt();
            break;
        }

        case PZ_SERVICE_CFG_EVT: /* Message about received CCCD write */
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

        case PZ_SERVICE_WRITE_EVT: /* Message about received value write */
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
}

#endif
