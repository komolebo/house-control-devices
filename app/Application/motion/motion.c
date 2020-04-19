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
#include <motion/motion_button_service.h>
#include <motion/motion_config_service.h>
#include <motion/motion_data_service.h>
#include <motion/motion_led_service.h>
#include <devinfoservice.h>
#include <device_common.h>
#include <icall.h>
#include <icall_ble_api.h>

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
const DeviceType DEVICE_TYPE = DEVICE_MOTION;
const uint8_t DEVICE_TYPE_NAME[] = "MOTION";

#define REL_VERSION_PATCH   "0"
#define REL_VERSION_MINOR   "1"
#define REL_VERSION_MAJOR   "0"

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
static void buttonDebounceSwiFxn(UArg buttonId);
static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId);
static void handleButtonPress(pzButtonState_t *pState);

/* Profile value change handlers */
static void ProjectZero_updateCharVal(CharacteristicData_t *pCharData);
static void LedService_ValueChangeHandler(
    CharacteristicData_t *pCharData);
static void ButtonService_CfgChangeHandler(
    CharacteristicData_t *pCharData);
static void DataService_ValueChangeHandler(
    CharacteristicData_t *pCharData);
static void ConfigService_ValueChangeHandler(
    CharacteristicData_t *pCharData);

static void LedService_ValueChangeCB(uint16_t connHandle,
                                     uint8_t paramID,
                                     uint16_t len,
                                     uint8_t *pValue);
static void DataService_ValueChangeCB(uint16_t connHandle,
                                      uint8_t paramID,
                                      uint16_t len,
                                      uint8_t *pValue);
static void ConfigService_ValueChangeCB(uint16_t connHandle,
                                        uint8_t paramID,
                                        uint16_t len,
                                        uint8_t *pValue);
static void ButtonService_CfgChangeCB(uint16_t connHandle,
                                      uint8_t paramID,
                                      uint16_t len,
                                      uint8_t *pValue);
static void DataService_CfgChangeCB(uint16_t connHandle,
                                    uint8_t paramID,
                                    uint16_t len,
                                    uint8_t *pValue);

/*********************************************************************
 * VARIABLES
 */
uint8_t advertData[ADVERT_DATA_NAME_DISPLAY_LEN + ADVERT_DATA_HEADER_LEN] =
{
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // complete name
    ADVERT_DATA_NAME_DISPLAY_LEN, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    ADVERT_DATA_NAME
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */
// LED Service callback handler.
// The type LED_ServiceCBs_t is defined in led_service.h
static LedServiceCBs_t LED_ServiceCBs =
{
    .pfnChangeCb = LedService_ValueChangeCB,  // Characteristic value change callback handler
    .pfnCfgChangeCb = NULL, // No notification-/indication enabled chars in LED Service
};

// Button Service callback handler.
// The type Button_ServiceCBs_t is defined in button_service.h
static ButtonServiceCBs_t Button_ServiceCBs =
{
    .pfnChangeCb = NULL,  // No writable chars in Button Service, so no change handler.
    .pfnCfgChangeCb = ButtonService_CfgChangeCB, // Noti/ind configuration callback handler
};

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
    .pfnCfgChangeCb = NULL, // No notification-/indication enabled chars in LED Service
};
/*********************************************************************
 * LOCAL VARIABLES
 */
/* Pin driver handles */
static PIN_Handle buttonPinHandle;
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;
static PIN_State ledPinState;

/*
 * Initial LED pin configuration table
 *   - LEDs Board_PIN_LED0 & Board_PIN_LED1 are off.
 */
PIN_Config ledPinTable[] = {
    Board_PIN_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_PIN_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_PIN_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

// Clock objects for debouncing the buttons
static Clock_Struct button0DebounceClock;
static Clock_Struct button1DebounceClock;
static Clock_Handle button0DebounceClockHandle;
static Clock_Handle button1DebounceClockHandle;

// State of the buttons
static uint8_t button0State = 0;
static uint8_t button1State = 0;


/*********************************************************************
 * FUNCTIONS
 */
void CustomDevice_hardwareInit(void)
{
    // ******************************************************************
    // Hardware initialization
    // ******************************************************************

    // Open LED pins
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle)
    {
        Log_error0("Error initializing board LED pins");
        Task_exit();
    }

    // Open button pins
    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if(!buttonPinHandle)
    {
        Log_error0("Error initializing button pins");
        Task_exit();
    }

    // Setup callback for button pins
    if(PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0)
    {
        Log_error0("Error registering button callback function");
        Task_exit();
    }

    // Create the debounce clock objects for Button 0 and Button 1
    button0DebounceClockHandle = Util_constructClock(&button0DebounceClock,
                                                     buttonDebounceSwiFxn, 50,
                                                     0,
                                                     0,
                                                     Board_PIN_BUTTON0);
    button1DebounceClockHandle = Util_constructClock(&button1DebounceClock,
                                                     buttonDebounceSwiFxn, 50,
                                                     0,
                                                     0,
                                                     Board_PIN_BUTTON1);
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
    LedService_AddService(selfEntity);
    ButtonService_AddService(selfEntity);
    DataService_AddService(selfEntity);
    ConfigService_AddService(selfEntity);

    // Register callbacks with the generated services that
    // can generate events (writes received) to the application
    LedService_RegisterAppCBs(&LED_ServiceCBs);
    ButtonService_RegisterAppCBs(&Button_ServiceCBs);
    DataService_RegisterAppCBs(&Data_ServiceCBs);
    ConfigService_RegisterAppCBs(&Config_ServiceCBs);

    // Placeholder variable for characteristic intialization
    uint8_t initVal[40] = {0};
    uint8_t initString[] = "This is a pretty long string, isn't it!";

    // Initalization of characteristics in LED_Service that can provide data.
    LedService_SetParameter(LS_LED0_ID, LS_LED0_LEN, initVal);
    LedService_SetParameter(LS_LED1_ID, LS_LED1_LEN, initVal);

    // Initalization of characteristics in Button_Service that can provide data.
    ButtonService_SetParameter(BS_BUTTON0_ID, BS_BUTTON0_LEN, initVal);
    ButtonService_SetParameter(BS_BUTTON1_ID, BS_BUTTON1_LEN, initVal);

    // Initalization of characteristics in Data_Service that can provide data.
    DataService_SetParameter(DS_STRING_ID, sizeof(initString), initString);
    DataService_SetParameter(DS_STREAM_ID, DS_STREAM_LEN, initVal);
}

/*
 *  Callbacks from Swi-context
 *****************************************************************************/
/*********************************************************************
 * @fn     buttonDebounceSwiFxn
 *
 * @brief  Callback from Clock module on timeout
 *
 *         Determines new state after debouncing
 *
 * @param  buttonId    The pin being debounced
 */
static void buttonDebounceSwiFxn(UArg buttonId)
{
    // Used to send message to app
    pzButtonState_t buttonMsg = { .pinId = buttonId };
    uint8_t sendMsg = FALSE;

    // Get current value of the button pin after the clock timeout
    uint8_t buttonPinVal = PIN_getInputValue(buttonId);

    // Set interrupt direction to opposite of debounced state
    // If button is now released (button is active low, so release is high)
    if(buttonPinVal)
    {
        // Enable negative edge interrupts to wait for press
        PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_NEGEDGE);
    }
    else
    {
        // Enable positive edge interrupts to wait for relesae
        PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_POSEDGE);
    }

    switch(buttonId)
    {
    case Board_PIN_BUTTON0:
        // If button is now released (buttonPinVal is active low, so release is 1)
        // and button state was pressed (buttonstate is active high so press is 1)
        if(buttonPinVal && button0State)
        {
            // Button was released
            buttonMsg.state = button0State = 0;
            sendMsg = TRUE;
        }
        else if(!buttonPinVal && !button0State)
        {
            // Button was pressed
            buttonMsg.state = button0State = 1;
            sendMsg = TRUE;
        }
        break;

    case Board_PIN_BUTTON1:
        // If button is now released (buttonPinVal is active low, so release is 1)
        // and button state was pressed (buttonstate is active high so press is 1)
        if(buttonPinVal && button1State)
        {
            // Button was released
            buttonMsg.state = button1State = 0;
            sendMsg = TRUE;
        }
        else if(!buttonPinVal && !button1State)
        {
            // Button was pressed
            buttonMsg.state = button1State = 1;
            sendMsg = TRUE;
        }
        break;
    }

    if(sendMsg == TRUE)
    {
        pzButtonState_t *pButtonState = ICall_malloc(sizeof(pzButtonState_t));
        if(pButtonState != NULL)
        {
            *pButtonState = buttonMsg;
            if(enqueueMsg(PZ_BUTTON_DEBOUNCED_EVT, pButtonState) != SUCCESS)
            {
              ICall_free(pButtonState);
            }
        }
    }
}

/*********************************************************************
 * @fn     buttonCallbackFxn
 *
 * @brief  Callback from PIN driver on interrupt
 *
 *         Sets in motion the debouncing.
 *
 * @param  handle    The PIN_Handle instance this is about
 * @param  pinId     The pin that generated the interrupt
 */
static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
    Log_info1("Button interrupt: %s",
              (uintptr_t)((pinId == Board_PIN_BUTTON0) ? "Button 0" : "Button 1"));

    // Disable interrupt on that pin for now. Re-enabled after debounce.
    PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);

    // Start debounce timer
    switch(pinId)
    {
    case Board_PIN_BUTTON0:
        Util_startClock((Clock_Struct *)button0DebounceClockHandle);
        break;
    case Board_PIN_BUTTON1:
        Util_startClock((Clock_Struct *)button1DebounceClockHandle);
        break;
    }
}

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
    case LED_SERVICE_SERV_UUID:
        LedService_SetParameter(pCharData->paramID, pCharData->dataLen,
                                pCharData->data);
        break;

    case BUTTON_SERVICE_SERV_UUID:
        ButtonService_SetParameter(pCharData->paramID, pCharData->dataLen,
                                   pCharData->data);
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
void LedService_ValueChangeHandler(
    CharacteristicData_t *pCharData)
{
    static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"
    util_arrtohex(pCharData->data, pCharData->dataLen,
                  pretty_data_holder, sizeof(pretty_data_holder),
                  UTIL_ARRTOHEX_NO_REVERSE);

    switch(pCharData->paramID)
    {
    case LS_LED0_ID:
        Log_info3("Value Change msg: %s %s: %s",
                  (uintptr_t)"LED Service",
                  (uintptr_t)"LED0",
                  (uintptr_t)pretty_data_holder);

        // Do something useful with pCharData->data here
        // -------------------------
        // Set the output value equal to the received value. 0 is off, not 0 is on
        PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, pCharData->data[0]);
        Log_info2("Turning %s %s",
                  (uintptr_t)ANSI_COLOR(FG_RED)"LED0"ANSI_COLOR(ATTR_RESET),
                  (uintptr_t)(pCharData->data[0] ? "on" : "off"));
        break;

    case LS_LED1_ID:
        Log_info3("Value Change msg: %s %s: %s",
                  (uintptr_t)"LED Service",
                  (uintptr_t)"LED1",
                  (uintptr_t)pretty_data_holder);

        // Do something useful with pCharData->data here
        // -------------------------
        // Set the output value equal to the received value. 0 is off, not 0 is on
        PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, pCharData->data[0]);
        Log_info2("Turning %s %s",
                  (uintptr_t)ANSI_COLOR(FG_GREEN)"LED1"ANSI_COLOR(ATTR_RESET),
                  (uintptr_t)(pCharData->data[0] ? "on" : "off"));
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
 * @fn      LedService_ValueChangeCB
 *
 * @brief   Callback for characteristic change when a peer writes to us
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void LedService_ValueChangeCB(uint16_t connHandle,
                                                 uint8_t paramID, uint16_t len,
                                                 uint8_t *pValue)
{
    // See the service header file to compare paramID with characteristic.
    Log_info1("(CB) LED Svc Characteristic value change: paramID(%d). "
              "Sending msg to app.", paramID);

    CharacteristicData_t *pValChange =
        ICall_malloc(sizeof(CharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = LED_SERVICE_SERV_UUID;
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
 * @fn      ButtonService_CfgChangeCB
 *
 * @brief   Callback for when a peer enables or disables the CCCD attribute,
 *          indicating they are interested in notifications or indications.
 *
 * @param   connHandle - connection handle
 *          paramID - the parameter ID maps to the characteristic written to
 *          len - length of the data written
 *          pValue - pointer to the data written
 */
static void ButtonService_CfgChangeCB(uint16_t connHandle,
                                                  uint8_t paramID, uint16_t len,
                                                  uint8_t *pValue)
{
    Log_info1("(CB) Button Svc Char config change paramID(%d). "
              "Sending msg to app.", paramID);

    CharacteristicData_t *pValChange =
        ICall_malloc(sizeof(CharacteristicData_t) + len);

    if(pValChange != NULL)
    {
        pValChange->svcUUID = BUTTON_SERVICE_SERV_UUID;
        pValChange->paramID = paramID;
        memcpy(pValChange->data, pValue, len);
        pValChange->dataLen = len;

        if(enqueueMsg(PZ_SERVICE_CFG_EVT, pValChange) != SUCCESS)
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
static void handleButtonPress(pzButtonState_t *pState)
{
    Log_info2("%s %s",
              (uintptr_t)(pState->pinId ==
                          Board_PIN_BUTTON0 ? "Button 0" : "Button 1"),
              (uintptr_t)(pState->state ?
                          ANSI_COLOR(FG_GREEN)"pressed"ANSI_COLOR(ATTR_RESET) :
                          ANSI_COLOR(FG_YELLOW)"released"ANSI_COLOR(ATTR_RESET)
                         ));

    // Update the service with the new value.
    // Will automatically send notification/indication if enabled.
    switch(pState->pinId)
    {
    case Board_PIN_BUTTON0:
        ButtonService_SetParameter(BS_BUTTON0_ID,
                                   sizeof(pState->state),
                                   &pState->state);
        break;
    case Board_PIN_BUTTON1:
        ButtonService_SetParameter(BS_BUTTON1_ID,
                                   sizeof(pState->state),
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
            pzButtonState_t *pButtonState = (pzButtonState_t *)pMsg->pData;
            handleButtonPress(pButtonState);
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
              case LED_SERVICE_SERV_UUID:
                  LedService_ValueChangeHandler(pCharData);
                  break;
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
