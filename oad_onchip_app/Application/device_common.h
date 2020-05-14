/*
 * device_common.h
 *
 *
 *  Created on: 17 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_DEVICE_COMMON_H_
#define APPLICATION_DEVICE_COMMON_H_

#include "board.h"
#include <string.h>
#include <stdint.h>
#include <bcomdef.h>
#include <icall_ble_api.h>
#include <icall.h>
#include "util.h"
#include <inc/hw_fcfg1.h>
#include <devinfoservice.h>
#include <ble_user_config.h>
#include <icall_ble_apimsg.h>

/*********************************************************************
 * MACROS
 */

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

#define Log_info0(x) {}

#define Log_info1(x, x1) {}

#define Log_info2(x, x1, x2) {}

#define Log_info3(x, x1, x2, x3) {}

#define Log_info4(x, x1, x2, x3, x4) {}

#define Log_info5(x, x1, x2, x3, x4, x5) {}



#define Log_error0(x) {}
#define Log_error1(x, x1) {}
#define Log_error2(x, x1, x2) {}
/*********************************************************************
 * TYPEDEFS
 */

typedef enum
{
    SBP_STATE_CHANGE_EVT,
    EVT_OAD_RESET,
    EVT_PASSCODE_NEEDED,    /* A pass-code/PIN is requested during pairing */
    EVT_SERVICE_WRITE,      /* A characteristic value has been written     */
    EVT_SERVICE_CFG,        /* A characteristic configuration has changed  */
    EVT_BUTTON_DEBOUNCED,   /* A button has been debounced with new value  */
    EVT_PAIRSTATE,          /* The pairing state is updated                */
    EVT_ADV,                /* A subscribed advertisement activity         */
    EVT_START_ADV,          /* Request advertisement start from task ctx   */
    EVT_SEND_PARAM_UPD,     /* Request parameter update req be sent        */
    EVT_CONN,               /* Connection Event End notice                 */
    EVT_TAMPER_CHANGED,     /* Tamper button state changed                 */
    EVT_LED_CLK,            /* LED blinking clock event                    */
    EVT_MOTION_CLK,         /* Motion timer expiration clock event         */
    EVT_GAP_CHANGE,
    // to handle later:
    EVT_INIT_DONE,
    EVT_CHECK_PRECOND,
    EVT_MODE_CHANGE,
    EVT_LED_CHANGE,
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

// Struct for messages sent to the application task
typedef struct
{
    uint8_t event;
    void    *pData;
} Msg_t;

// Struct for messages about characteristic data
typedef struct
{
    uint16_t svcUUID; // UUID of the service
    uint16_t dataLen; //
    uint8_t paramID; // Index of the characteristic
    uint8_t data[]; // Flexible array member, extended to malloc - sizeof(.)
} CharacteristicData_t;

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
  uint8_t *pData;   // Event payload
} sbpEvt_t;

/*********************************************************************
 *  EXTERNAL VARIABLES
 */
//extern const uint8_t DevicesName[DEVICE_COUNT][DEVINFO_STR_ATTR_LEN];

/*********************************************************************
 *  EXTERNAL FUNCTIONS
 */
uint8_t SimplePeripheral_enqueueMsg(uint8_t event, uint8_t state, uint8_t *pData);
bStatus_t enqueueMsg(uint8_t event, void *pData);
bool isConnected(void);

/*********************************************************************
 * FUNCTIONS
 */

#endif /* APPLICATION_DEVICE_COMMON_H_ */
