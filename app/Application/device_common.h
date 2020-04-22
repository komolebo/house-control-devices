/*
 * device_common.h
 *
 *
 *  Created on: 17 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_DEVICE_COMMON_H_
#define APPLICATION_DEVICE_COMMON_H_

#include <stdint.h>
#include <bcomdef.h>
#include <devinfoservice.h>
#include <icall_ble_api.h>

/*********************************************************************
 * CONSTANTS
 */
#define DEVICE_STATE_ENABLED                (1)
#define DEVICE_STATE_DISABLED               (0)

#define MOTION_SENSITIVITY_3M               (0)
#define MOTION_SENSITIVITY_4M               (1)
#define MOTION_SENSITIVITY_5M               (2)
#define MOTION_SENSITIVITY_6M               (3)
#define MOTION_SENSITIVITY_7M               (4)

/*********************************************************************
 * TYPEDEFS
 */
// Struct for message about button state
typedef struct
{
    PIN_Id pinId;
    uint8_t state;
} ButtonState_t;


/*********************************************************************
 * MACROS
 */
#define VAR_NAME_TO_STR(var_name) (#var_name)

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Advert data header len to determine name string address
#define ADVERT_DATA_HEADER_LEN  (5)

// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
#define PZ_SERVICE_WRITE_EVT     0  /* A characteristic value has been written     */
#define PZ_SERVICE_CFG_EVT       1  /* A characteristic configuration has changed  */
#define PZ_UPDATE_CHARVAL_EVT    2  /* Request from ourselves to update a value    */
#define PZ_BUTTON_DEBOUNCED_EVT  3  /* A button has been debounced with new value  */
#define PZ_PAIRSTATE_EVT         4  /* The pairing state is updated                */
#define PZ_PASSCODE_EVT          5  /* A pass-code/PIN is requested during pairing */
#define PZ_ADV_EVT               6  /* A subscribed advertisement activity         */
#define PZ_START_ADV_EVT         7  /* Request advertisement start from task ctx   */
#define PZ_SEND_PARAM_UPD_EVT    8  /* Request parameter update req be sent        */
#define PZ_CONN_EVT              9  /* Connection Event End notice                 */

#define UTIL_ARRTOHEX_REVERSE     1
#define UTIL_ARRTOHEX_NO_REVERSE  0
/*********************************************************************
 * TYPEDEFS
 */
enum DeviceType
{
    DEVICE_MOTION,
    DEVICE_SMOKE,
    DEVICE_GAS,
    DEVICE_COUNT
};

typedef enum DeviceType DeviceType;

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
char * util_arrtohex(uint8_t const *src,
                            uint8_t src_len,
                            uint8_t       *dst,
                            uint8_t dst_len,
                            uint8_t reverse);


#endif /* APPLICATION_DEVICE_COMMON_H_ */
