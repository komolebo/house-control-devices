/*
 * device_common.h
 *
 *
 *  Created on: 17 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_DEVICE_COMMON_H_
#define APPLICATION_DEVICE_COMMON_H_

#include <string.h>
#include <stdint.h>
#include <bcomdef.h>
#include <icall_ble_api.h>
#include <icall.h>
#include "util.h"
#include <inc/hw_fcfg1.h>
#include <devinfoservice.h>


/*********************************************************************
 * MACROS
 */

// Minimum connection interval (units of 1.25ms, 80=100ms) for parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     12

// Maximum connection interval (units of 1.25ms, 800=1000ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     36

// Slave latency to use for parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) for parameter update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          200

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// Connection interval conversion rate to miliseconds
#define CONN_INTERVAL_MS_CONVERSION           1.25

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_PASS_TO_APP

// Delay (in ms) after connection establishment before sending a parameter update requst
#define PZ_SEND_PARAM_UPDATE_DELAY            6000

// Spin if the expression is not true
#define APP_ASSERT(expr) if(!(expr)) {spinForever();}


/*********************************************************************
 * CONSTANTS
 */
#define DEVICE_STATE_ENABLED                (1)
#define DEVICE_STATE_DISABLED               (0)

#define LED_HIGH                            (1)
#define LED_LOW                             (0)

#define MOTION_SENSITIVITY_3M               (0)
#define MOTION_SENSITIVITY_4M               (1)
#define MOTION_SENSITIVITY_5M               (2)
#define MOTION_SENSITIVITY_6M               (3)
#define MOTION_SENSITIVITY_7M               (4)

#define TAMPER_PIN                          (Board_PIN_BUTTON0)
#define LED_PIN                             (Board_PIN_RLED)


// Minimum connection interval (units of 1.25ms, 80=100ms) for parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     12

// Maximum connection interval (units of 1.25ms, 800=1000ms) for  parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     36

// Slave latency to use for parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) for parameter update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          200

// Delay (in ms) after connection establishment before sending a parameter update requst
#define SEND_PARAM_UPDATE_DELAY            6000

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// Connection interval conversion rate to miliseconds
#define CONN_INTERVAL_MS_CONVERSION           1.25

// Pass parameter updates to the app for it to decide.
#define DEFAULT_PARAM_UPDATE_REQ_DECISION     GAP_UPDATE_REQ_PASS_TO_APP

/*********************************************************************
 * MACROS
 */
#define VAR_NAME_TO_STR(var_name) (#var_name)

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Advert data header len to determine name string address
#define ADVERT_DATA_HEADER_LEN  (5)

#define UTIL_ARRTOHEX_REVERSE     1
#define UTIL_ARRTOHEX_NO_REVERSE  0
/*********************************************************************
 * TYPEDEFS
 */
typedef enum
{
    EVT_SERVICE_WRITE,      /* A characteristic value has been written     */
    EVT_SERVICE_CFG,        /* A characteristic configuration has changed  */
    EVT_UPDATE_CHARVAL,     /* Request from ourselves to update a value    */
    EVT_BUTTON_DEBOUNCED,   /* A button has been debounced with new value  */
    EVT_PAIRSTATE,          /* The pairing state is updated                */
    EVT_PASSCODE,           /* A pass-code/PIN is requested during pairing */
    EVT_ADV,                /* A subscribed advertisement activity         */
    EVT_START_ADV,          /* Request advertisement start from task ctx   */
    EVT_SEND_PARAM_UPD,     /* Request parameter update req be sent        */
    EVT_CONN,               /* Connection Event End notice                 */
    EVT_TAMPER_CHANGED,     /* Tamper button state changed                 */
    EVT_LED_CLK,            /* LED blinking clock event                    */
    EVT_MOTION_CLK,         /* Motion timer expiration clock event         */
    // to handle later:
    EVT_INIT_DONE,
    EVT_CHECK_PRECOND,
    EVT_MODE_CHANGE,
    EVT_DISCONN,
    EVT_DISABLE,
    EVT_CALIBRATE,
    EVT_MEASURE,
    EVT_DETECT,
} appEvt_t;

typedef enum
{
    DEVICE_MOTION,
    DEVICE_SMOKE,
    DEVICE_GAS,
    DEVICE_COUNT
} DeviceType;

// List element for parameter update and PHY command status lists
typedef struct
{
    List_Elem elem;
    uint16_t *connHandle;
} pzConnHandleEntry_t;

// Connected device information
typedef struct
{
    uint16_t connHandle;                    // Connection Handle
    Clock_Struct* pUpdateClock;             // pointer to clock struct
    bool phyCngRq;                          // Set to true if PHY change request is in progress
    uint8_t currPhy;                        // The active PHY for a connection
    uint8_t rqPhy;                          // The requested PHY for a connection
    uint8_t phyRqFailCnt;                   // PHY change request fail count
} pzConnRec_t;

// Struct for message about button state
typedef struct
{
    PIN_Id pinId;
    uint8_t state;
} ButtonState_t;

// Struct for messages about characteristic data
typedef struct
{
    uint16_t svcUUID; // UUID of the service
    uint16_t dataLen; //
    uint8_t paramID; // Index of the characteristic
    uint8_t data[]; // Flexible array member, extended to malloc - sizeof(.)
} CharacteristicData_t;

// Struct for messages sent to the application task
typedef struct
{
    uint8_t event;
    void    *pData;
} Msg_t;

// Struct for message about a pending parameter update request.
typedef struct
{
    uint16_t connHandle;
} SendParamReq_t;

/*********************************************************************
 *  EXTERNAL VARIABLES
 */
extern const uint8_t DevicesName[DEVICE_COUNT][DEVINFO_STR_ATTR_LEN];

/*********************************************************************
 *  EXTERNAL FUNCTIONS
 */
extern status_t enqueueMsg(uint8_t event, void *pData);


/*********************************************************************
 * FUNCTIONS
 */
void spinForever(void);
char * util_arrtohex(uint8_t const *src,
                     uint8_t src_len,
                     uint8_t       *dst,
                     uint8_t dst_len,
                     uint8_t reverse);

/* Connection handling functions */
bool isConnected(void);
uint8_t getConnIndex(uint16_t connHandle);
uint8_t clearConnListEntry(uint16_t connHandle);
uint8_t addConn(uint16_t connHandle);
uint8_t removeConn(uint16_t connHandle);

void sendParamUpdate(uint16_t connHandle);
void updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
void paramUpdClockHandler(UArg arg);
void paramUpdClockHandler(UArg arg);
void handleUpdateLinkEvent(gapLinkUpdateEvent_t *pEvt);

#endif /* APPLICATION_DEVICE_COMMON_H_ */
