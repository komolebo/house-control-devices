/******************************************************************************

 @file  simple_peripheral_oad_onchip.c

 @brief This file contains the OAD sample application based on
        simple_peripheral for use with the CC2650 Bluetooth Low Energy
        Protocol Stack.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2017-2020, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#ifdef LED_DEBUG
#include <ti/drivers/PIN.h>
#endif //LED_DEBUG

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "devinfoservice.h"
#include "ll_common.h"

// Used for imgHdr_t structure
#include "oad_image_header.h"
// Used for OAD Reset Service APIs
#include "oad_reset_service.h"

// Pull in common OAD structs
#include <profiles/oad/cc26xx/oad.h>
#include <profiles/oad/cc26xx/oad_util.h>

// Needed for ECC constants
#include <common/cc26xx/ecc/ECCROMCC26XX.h>

// Needed for HAL_SYSTEM_RESET()
#include "hal_mcu.h"

#include "peripheral.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include "board.h"

#include "simple_peripheral_oad_onchip.h"

#include "ble_user_config.h"

#include <custom_device_if.h>
#include <profiles_if.h>
#include "device_common.h"
/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Define the default pairing behavior
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               5000

// Offset into the scanRspData string the software version info is stored
#define OAD_SOFT_VER_OFFSET                   15


// Task configuration
#define SBP_TASK_PRIORITY                     1

// Warning! To optimize RAM, task stack size must be a multiple of 8 bytes
#ifndef SBP_TASK_STACK_SIZE
  #if defined (__IAR_SYSTEMS_ICC__)
    #define SBP_TASK_STACK_SIZE                   1000
  #elif defined __TI_COMPILER_VERSION || defined __TI_COMPILER_VERSION__
    #define SBP_TASK_STACK_SIZE                   1000
  #else
    #error "Unknown Compiler"
   #endif
#endif

// Internal Events for RTOS application
#define SBP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBP_PERIODIC_EVT                      Event_Id_00
#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               SBP_PERIODIC_EVT)

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(RegisterCause) (connectionEventRegisterCauseBitMap |= RegisterCause )
// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(RegisterCause) (connectionEventRegisterCauseBitMap &= (~RegisterCause) )
// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)
// Gets whether the RegisterCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(RegisterCause) (connectionEventRegisterCauseBitMap & RegisterCause )

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

extern const imgHdr_t _imageHdr;

// BIM variable
#ifdef __TI_COMPILER_VERSION__
  #pragma location = BIM_VAR_ADDR
  #pragma NOINIT(_bim_var)
  uint32_t _bim_var;
#elif __IAR_SYSTEMS_ICC__
  __no_init uint32_t _bim_var @ BIM_VAR_ADDR;
#endif

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbpTask;

#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(sbpTaskStack, 8)
#elif __IAR_SYSTEMS_ICC__
#pragma data_alignment=8
#endif    //__TI_COMPILER_VERSION__
uint8_t sbpTaskStack[SBP_TASK_STACK_SIZE];


// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x12,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'M',
  'O',
  'T',
  'I',
  'O',
  'N',
  ' ',
  'O',
  'A',
  'D',
  ' ',
  ' ',
  'V',
  ' ', // These 4 octets are placeholders for the SOFTVER field
  ' ', // which will be updated at init time
  ' ',
  ' ',

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 10ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 10ms
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x05,  // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(OAD_RESET_SERVICE_UUID),
  HI_UINT16(OAD_RESET_SERVICE_UUID),
  LO_UINT16(0xFF00),
  HI_UINT16(0xFF01)
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "SBP OAD on-chip";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;
static uint8_t verifyStatus = OAD_INCOMPATIBLE_IMAGE;


#if defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED)
// Flag to be stored in NV that tracks whether service changed
// indications needs to be sent out
static uint32_t  sendSvcChngdOnNextBoot = FALSE;
#endif // ( defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED) )

// Variable used to store the number of messages pending once OAD completes
// The application cannot reboot until all pending messages are sent
static uint8_t numPendingMsgs = 0;

#ifdef LED_DEBUG
// Pin table for LED debug pins
static const PIN_Config sbpLedPins[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
#endif //LED_DEBUG

// State variable for debugging LEDs
PIN_State  sbpLedState;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimplePeripheral_init( void );
static void SimplePeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimplePeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimplePeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimplePeripheral_clockHandler(UArg arg);
static void SimplePeripheral_sendAttRsp(void);
static void SimplePeripheral_freeAttRsp(uint8_t status);
static void SimplePeripheral_stateChangeCB(gaprole_States_t newState);

static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void SimplePeripheral_processOadResetEvt(oadResetWrite_t *resetEvt);
static uint8_t SimplePeripheral_processL2CAPMsg(l2capSignalEvent_t *pMsg);
void SimplePeripheral_processOadResetWriteCB(uint16_t connHandle,
                                                uint8_t *payload,
                                                uint16_t len);
static void SimplePeripheral_processPasscode(gapPasskeyNeededEvent_t *pData);
static void SimplePeripheral_passcodeCB(uint8_t *deviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);


/*********************************************************************
 * EXTERN FUNCTIONS
 */
#if 0
extern void AssertHandler(uint8_t assertCause, uint8_t assertSubcause);
#endif
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimplePeripheral_gapRoleCBs =
{
  SimplePeripheral_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t SimplePeripheral_BondMgrCBs =
{
  SimplePeripheral_passcodeCB, // Passcode callback,
  NULL                         // Pairing / Bonding state Callback (not used by application)
};

static oadResetWriteCB_t SimplePeripheral_oadResetCBs =
{
  SimplePeripheral_processOadResetWriteCB // Write Callback.
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * The following typedef and global handle the registration to connection event
 */
typedef enum
{
   NOT_REGISTER       = 0,
   FOR_AOA_SCAN       = 1,
   FOR_ATT_RSP        = 2,
   FOR_AOA_SEND       = 4,
   FOR_TOF_SEND       = 8,
   FOR_OAD_SEND       = 0x10,
}connectionEventRegisterCause_u;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t       connectionEventRegisterCauseBitMap = NOT_REGISTER; //see connectionEventRegisterCause_u

/*********************************************************************
 * @fn      SimplePeripheral_RegistertToAllConnectionEvent()
 *
 * @brief   register to receive connection events for all the connection
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t SimplePeripheral_RegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  // in case  there is no registration for the connection event, make the registration
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = GAP_RegisterConnEventCb(SimplePeripheral_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
  }
  if(status == SUCCESS)
  {
    //add the reason bit to the bitamap.
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
}

/*********************************************************************
 * @fn      SimplePeripheral_UnRegistertToAllConnectionEvent()
 *
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t SimplePeripheral_UnRegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
  // in case  there is no more registration for the connection event than unregister
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    GAP_RegisterConnEventCb(SimplePeripheral_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
  }

  return(status);
}

/*********************************************************************
 * @fn      SimplePeripheral_createTask
 *
 * @brief   Task creation function for the OAD User App.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimplePeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, SimplePeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_init(void)
{
  // ******************************************************************
  // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SimplePeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, FALSE, SBP_PERIODIC_EVT);

  // For on-chip OAD stack-only download must be followed by app-only
  // So the current app must always be linked to the right stack
  // use this info to read in the stack version
  imgHdr_t *stackImageHeader = (imgHdr_t *)(_imageHdr.boundarySeg.stackStartAddr);

  // Read in the OAD Software version
  uint8_t swVer[OAD_SW_VER_LEN] = {stackImageHeader->fixedHdr.softVer[0],
                                    stackImageHeader->fixedHdr.softVer[1],
                                    _imageHdr.fixedHdr.softVer[2],
                                    _imageHdr.fixedHdr.softVer[3]};

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    // Setup the dyanmic portion of the scanRspData
    scanRspData[OAD_SOFT_VER_OFFSET] = swVer[0];
    scanRspData[OAD_SOFT_VER_OFFSET + 1] = swVer[1];
    scanRspData[OAD_SOFT_VER_OFFSET + 2] = swVer[2];
    scanRspData[OAD_SOFT_VER_OFFSET + 3] = swVer[3];

    // Set scanRspData
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;
    uint8_t scMode = GAPBOND_SECURE_CONNECTION_ALLOW;

    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    GAPBondMgr_SetParameter(GAPBOND_SECURE_CONNECTION, sizeof(uint8_t), &scMode);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
//  DevInfo_AddService();                        // Device Information Service

  CustomDevice_init(selfEntity);

  Reset_addService((oadUsrAppCBs_t *)&SimplePeripheral_oadResetCBs);

  // Start the Device
  VOID GAPRole_StartDevice(&SimplePeripheral_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&SimplePeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  uint8_t versionStr[OAD_SW_VER_LEN + 1];

  memcpy(versionStr, swVer, OAD_SW_VER_LEN);

  // Add in Null terminator
  versionStr[OAD_SW_VER_LEN] = NULL;


#ifdef LED_DEBUG
  // Open the LED debug pins
  if (!PIN_open(&sbpLedState, sbpLedPins))
  {
  }
  else
  {
    PIN_Id activeLed;
    uint8_t numBlinks = 9;
    uint8_t cnt=0;
    for(; cnt < 10; ++cnt)
    {
      activeLed = Board_LED0;

      PIN_setOutputValue(&sbpLedState, activeLed, !PIN_getOutputValue(activeLed));

      // Sleep for 100ms, sys-tick for BLE-Stack is 10us,
      // Task sleep is in # of ticks
      Task_sleep(10000);

    }
    for(cnt=0; cnt<numBlinks; ++cnt)
    {
      activeLed = Board_LED1;

      PIN_setOutputValue(&sbpLedState, activeLed, !PIN_getOutputValue(activeLed));

      // Sleep for 100ms, sys-tick for BLE-Stack is 10us,
      // Task sleep is in # of ticks
      Task_sleep(10000);

    }

    // Close the pins after using
    PIN_close(&sbpLedState);
  }
#endif //LED_DEBUG

#if defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED)
  /*
   * When switching from persistent app back to the user application for the
   * for the first time after an OAD the device must send a service changed
   * indication. This will cause any peers to rediscover services.
   *
   * To prevent sending a service changed IND on every boot, a flag is stored
   * in NV to determine whether or not the service changed IND needs to be
   * sent
   */
  uint8_t status = osal_snv_read(BLE_NVID_CUST_START,
                                  sizeof(sendSvcChngdOnNextBoot),
                                  (uint8 *)&sendSvcChngdOnNextBoot);
  if(status != SUCCESS)
  {
    /*
     * On first boot the NV item will not have yet been initialzed, and the read
     * will fail. Do a write to set the initial value of the flash in NV
     */
     osal_snv_write(BLE_NVID_CUST_START, sizeof(sendSvcChngdOnNextBoot),
                    (uint8 *)&sendSvcChngdOnNextBoot);
  }
#endif // ( defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED) )
}

/*********************************************************************
 * @fn      SimplePeripheral_taskFxn
 *
 * @brief   Application task entry point for the OAD User App.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimplePeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimplePeripheral_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, SBP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8_t safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = SimplePeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SBP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {

          // Get the first message from the Queue
          sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);

          if (pMsg)
          {
            // Process message.
            SimplePeripheral_processAppMsg(pMsg);

            if (pMsg->pData != NULL)
            {
              // Free the Queue payload if there is one
              ICall_free(pMsg->pData);
            }

            // Free the space from the message.
            ICall_free(pMsg);
          }

        }
      }

      if (events & SBP_PERIODIC_EVT)
      {
        Util_startClock(&periodicClock);
      }
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimplePeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {

        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
#if 0
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
#endif
            break;

          default:
            break;
        }
      }
      break;

      case L2CAP_SIGNAL_EVENT:
        // Process L2CAP signal
        safeToDealloc = SimplePeripheral_processL2CAPMsg((l2capSignalEvent_t *)pMsg);
        break;

      default:
        // do nothing
        break;

    }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processL2CAPMsg
 *
 * @brief   Process L2CAP messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processL2CAPMsg(l2capSignalEvent_t *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  static bool firstRun = TRUE;

  switch (pMsg->opcode)
  {
    case L2CAP_NUM_CTRL_DATA_PKT_EVT:
    {
      /*
       * We cannot reboot the device immediately after receiving
       * the enable command, we must allow the stack enough time
       * to process and respond to the OAD_EXT_CTRL_ENABLE_IMG
       * command. This command will determine the number of
       * packets currently queued up by the LE controller.
       * BIM var is already set via OadPersistApp_processOadWriteCB
       */
      if(firstRun)
      {
        firstRun = false;

        // We only want to set the numPendingMsgs once
        numPendingMsgs = MAX_NUM_PDU - pMsg->cmd.numCtrlDataPktEvt.numDataPkt;

        // Wait the number of connection events
        SimplePeripheral_RegistertToAllConnectionEvent (FOR_OAD_SEND);
      }

      break;
    }
    default:
      break;
  }

  // It's safe to free the incoming message
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if( SimplePeripheral_RegistertToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS)
    {
      // First free any pending response
      SimplePeripheral_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport)
{

  if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
  {
    // The GATT server might have returned a blePending as it was trying
    // to process an ATT Response. Now that we finished with this
    // connection event, let's try sending any remaining ATT Responses
    // on the next connection event.
    // Try to retransmit pending ATT Response (if any)
    SimplePeripheral_sendAttRsp();
  }
  if( CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_OAD_SEND))
  {
    // Wait until all pending messages are sent
    if((verifyStatus == OAD_SUCCESS) && (numPendingMsgs == 0))
    {
#if defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED)
      uint8_t status ;
      // Store the flag to indicate that a service changed IND will
      // be sent at the next boot
      sendSvcChngdOnNextBoot = TRUE;

      status = osal_snv_write(BLE_NVID_CUST_START,
                                      sizeof(sendSvcChngdOnNextBoot),
                                      (uint8 *)&sendSvcChngdOnNextBoot);
      if(status != SUCCESS)
      {
      }
#endif // ( defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED) )

      // Reset the system
      HAL_SYSTEM_RESET();
    }
    numPendingMsgs--;
  }
}
/*********************************************************************
 * @fn      SimplePeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimplePeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      SimplePeripheral_UnRegistertToAllConnectionEvent (FOR_ATT_RSP);
      // We're done with the response message
      SimplePeripheral_freeAttRsp(status);
    }
    else
    {
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimplePeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimplePeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
    {
      SimplePeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;
    }

    case EVT_OAD_RESET:
    {
      // Convert generic pData pointer to oadResetWrite_t
      oadResetWrite_t *oadResetEvt  = (oadResetWrite_t *)(pMsg->pData);

      // Process the reset event
      SimplePeripheral_processOadResetEvt(oadResetEvt);
      break;
    }

    case EVT_PASSCODE_NEEDED:
    {
      SimplePeripheral_processPasscode((gapPasskeyNeededEvent_t*)pMsg->pData);
      // Free the app data
      break;
    }
    case EVT_CONN:
    {
        SimplePeripheral_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));
        break;
    }

    default:
      // Do nothing.
      break;
  }

  CustomDevice_processApplicationMessage(pMsg);
}

/*********************************************************************
 * @fn      SimplePeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimplePeripheral_processStateChangeEvt(gaprole_States_t newState)
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        enqueueMsg(EVT_INIT_DONE, NULL);
      }
      break;

    case GAPROLE_ADVERTISING:
      break;

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;
        uint16_t connHandle = 0;

        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);

        Util_startClock(&periodicClock);

        numActive = linkDB_NumActive();

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        }

#if defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED)
        if(sendSvcChngdOnNextBoot == TRUE)
        {
          GAPBondMgr_ServiceChangeInd( connHandle, TRUE);

          sendSvcChngdOnNextBoot = FALSE;
        }
#endif // ( defined(GAP_BOND_MGR) && !defined(GATT_NO_SERVICE_CHANGED) )
        SimplePeripheral_enqueueMsg(EVT_CONN, 0, NULL);
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      SimplePeripheral_freeAttRsp(bleNotConnected);

      // Clear remaining lines
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimplePeripheral_freeAttRsp(bleNotConnected);

      break;

    case GAPROLE_ERROR:
        SimplePeripheral_enqueueMsg(EVT_DISCONN, 0, NULL);
      break;

    default:
      break;
  }

  enqueueMsg(EVT_GAP_CHANGE, NULL);
}

/*********************************************************************
* @fn      SimplePeripheral_processPasscode
*
* @brief   Process the Passcode request.
*
* @return  none
*/
static void SimplePeripheral_processPasscode(gapPasskeyNeededEvent_t *pData)
{
  // Use static passcode
  uint32_t passcode = 123456;
  // Send passcode to GAPBondMgr
  GAPBondMgr_PasscodeRsp(pData->connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      SimplePeripheral_processOadResetEvt
 *
 * @brief   Process a write request to the OAD reset service
 *
 * @param   resetEvt - The oadResetWrite_t struct containing reset data
 *
 * @return  None.
 */
static void SimplePeripheral_processOadResetEvt(oadResetWrite_t *resetEvt)
{
  /* We cannot reboot the device immediately after receiving
   * the enable command, we must allow the stack enough time
   * to process and responsd to the OAD_EXT_CTRL_ENABLE_IMG
   * command. The current implementation will wait one cxn evt
   */
  // Register for L2CAP Flow Control Events
  L2CAP_RegisterFlowCtrlTask(selfEntity);

  /* First, disable pairing for the duration of the sign/verify procedure
   * This is done because the BLE-stack may use the ECC code during pairing for
   * ECC key generation
   */
  uint8_t pairingMode = GAPBOND_PAIRING_MODE_NO_PAIRING;
  GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(pairingMode),
                          &pairingMode);
  uint8_t cmdID = resetEvt->payload[0];
  
  verifyStatus = OAD_SUCCESS; /* fake verify status */
  
  // Set the bim_var based on the command ID
  if(cmdID == OAD_RESET_CMD_START_OAD)
  {
    // Set the BIM variable to jump to persistent application
    _bim_var = 0x00000001;
  }
  else
  {
    // Set BIM variable to jump back to user application
    _bim_var = 0x00000101;
  }

  // re-enable paring
  pairingMode = DEFAULT_PAIRING_MODE;
  GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(pairingMode),
                          &pairingMode);
}

/*********************************************************************
 * Callback Functions - These run in the calling thread's context
 *********************************************************************/

 /*********************************************************************
* @fn      SimplePeripheral_passcodeCB
*
* @brief   Passcode callback.
*
* @param   deviceAddr - pointer to device address
*
* @param   connHandle - the connection handle
*
* @param   uiInputs - pairing User Interface Inputs
*
* @param   uiOutputs - pairing User Interface Outputs
*
* @param   numComparison - numeric Comparison 20 bits
*
* @return  none
*/
static void SimplePeripheral_passcodeCB(uint8_t *deviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  gapPasskeyNeededEvent_t *pData;

  // Allocate space for the passcode event.
  if ((pData = ICall_malloc(sizeof(gapPasskeyNeededEvent_t))))
  {
    memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
    pData->connectionHandle = connHandle;
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    SimplePeripheral_enqueueMsg(EVT_PASSCODE_NEEDED, NULL,
                                    (uint8_t *) pData);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimplePeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimplePeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_processOadResetWriteCB
 *
 * @brief   Process a write request to the OAD reset service
 *
 * @param   connHandle - the connection Handle this request is from.
 * @param   bim_var    - bim_var to set before resetting.
 *
 * @return  None.
 */
void SimplePeripheral_processOadResetWriteCB(uint16_t connHandle,
                                                uint8_t *payload,
                                                uint16_t len)
{
    // Allocate memory for OAD EVT payload, the app task must free this later
    oadResetWrite_t *oadResetWriteEvt = ICall_malloc(sizeof(oadResetWrite_t));
    if(oadResetWriteEvt)
    {
        oadResetWriteEvt->connHandle = connHandle;
        oadResetWriteEvt->payload = payload;
        oadResetWriteEvt->len = len;

        // This function will enqueue the messsage and wake the application
        SimplePeripheral_enqueueMsg(EVT_OAD_RESET, 0, (uint8_t *)oadResetWriteEvt);
    }
}

/*********************************************************************
 * SWI Functions - These functions run at higher priority than any task
 *********************************************************************/

/*********************************************************************
 * @fn      SimplePeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimplePeripheral_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}

/*********************************************************************
 * @fn      SimplePeripheral_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
    // Enqueue the event for processing in the app context.
    if (SimplePeripheral_enqueueMsg(EVT_CONN, 0, (uint8_t *) pReport) == FALSE)
    {
        ICall_free(pReport);
    }
}
/*********************************************************************
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
uint8_t SimplePeripheral_enqueueMsg(uint8_t event, uint8_t state,
                                           uint8_t *pData)
{
  sbpEvt_t *pMsg = ICall_malloc(sizeof(sbpEvt_t));

  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;
    pMsg->pData = pData;

    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}
/*********************************************************************/
