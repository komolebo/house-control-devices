/*
 * led_service.c
 *
 *  Created on: 18 ���. 2020 �.
 *      Author: Oleh
 */


/*********************************************************************
 * INCLUDES
 */
#if MOTION_COMPILE

#include <string.h>

//#include <xdc/runtime/Log.h> // Comment this in to use xdc.runtime.Log
#include <uartlog/UartLog.h>  // Comment out if using xdc Log

#include <icall.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include <services/led_service.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// LED_Service Service UUID
CONST uint8_t LedServiceUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(LED_SERVICE_SERV_UUID), HI_UINT16(LED_SERVICE_SERV_UUID)
};

// MODE UUID
CONST uint8_t ls_MODEUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(LS_MODE_UUID), HI_UINT16(LS_MODE_UUID)
};

// LED1 UUID
CONST uint8_t ls_LED1UUID[ATT_UUID_SIZE] =
{
    BASE128_FROM_UINT16(LS_LED1_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static LedServiceCBs_t *pAppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Service declaration
static CONST gattAttrType_t LedServiceDecl = { ATT_BT_UUID_SIZE, LedServiceUUID };

// Characteristic "MODE" Properties (for declaration)
static uint8_t ls_ModeProps = GATT_PROP_READ | GATT_PROP_WRITE |
                              GATT_PROP_WRITE_NO_RSP; // TODO: NO_RSP?

// Characteristic "MODE" Value variable
static uint8_t ls_ModeVal[LS_MODE_LEN] = { 0x01 };

// Length of data in characteristic "MODE" Value variable, initialized to minimal size.
static uint16_t ls_ModeValLen = LS_MODE_LEN_MIN;

// Characteristic "LED1" Properties (for declaration)
static uint8_t ls_LED1Props = GATT_PROP_READ | GATT_PROP_WRITE |
                              GATT_PROP_WRITE_NO_RSP;

// Characteristic "LED1" Value variable
static uint8_t ls_LED1Val[LS_LED1_LEN] = {0};

// Length of data in characteristic "LED1" Value variable, initialized to minimal size.
static uint16_t ls_LED1ValLen = LS_LED1_LEN_MIN;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t LED_ServiceAttrTbl[] =
{
    // LED_Service Service Declaration
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)&LedServiceDecl
    },
    // Mode Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &ls_ModeProps
    },
    // Mode Characteristic Value
    {
        { ATT_BT_UUID_SIZE, ls_MODEUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        ls_ModeVal
    },
    // LED1 Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &ls_LED1Props
    },
    // LED1 Characteristic Value
    {
        { ATT_UUID_SIZE, ls_LED1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        ls_LED1Val
    },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t LED_Service_ReadAttrCB(uint16_t connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8_t *pValue,
                                        uint16_t *pLen,
                                        uint16_t offset,
                                        uint16_t maxLen,
                                        uint8_t method);
static bStatus_t LED_Service_WriteAttrCB(uint16_t connHandle,
                                         gattAttribute_t *pAttr,
                                         uint8_t *pValue,
                                         uint16_t len,
                                         uint16_t offset,
                                         uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t LED_ServiceCBs =
{
    LED_Service_ReadAttrCB, // Read callback function pointer
    LED_Service_WriteAttrCB, // Write callback function pointer
    NULL                     // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * LedService_AddService- Initializes the LedService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t LedService_AddService(uint8_t rspTaskId)
{
    uint8_t status;

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(LED_ServiceAttrTbl,
                                         GATT_NUM_ATTRS(LED_ServiceAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &LED_ServiceCBs);
    Log_info2("Registered service, %d attributes, status 0x%02x",
              GATT_NUM_ATTRS(
                  LED_ServiceAttrTbl), status);
    return(status);
}

/*
 * LedService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t LedService_RegisterAppCBs(LedServiceCBs_t *appCallbacks)
{
    if(appCallbacks)
    {
        pAppCBs = appCallbacks;
        Log_info1("Registered callbacks to application. Struct %p",
                  (uintptr_t)appCallbacks);
        return(SUCCESS);
    }
    else
    {
        Log_warning0("Null pointer given for app callbacks.");
        return(FAILURE);
    }
}

/*
 * LedService_SetParameter - Set a LedService parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t LedService_SetParameter(uint8_t param, uint16_t len, void *value)
{
    bStatus_t ret = SUCCESS;
    uint8_t  *pAttrVal;
    uint16_t *pValLen;
    uint16_t valMinLen;
    uint16_t valMaxLen;

    switch(param)
    {
    case LS_LED1_ID:
        pAttrVal = ls_LED1Val;
        pValLen = &ls_LED1ValLen;
        valMinLen = LS_LED1_LEN_MIN;
        valMaxLen = LS_LED1_LEN;
        Log_info2("SetParameter : %s len: %d", (uintptr_t)"LED1", len);
        break;

    default:
        Log_error1("SetParameter: Parameter #%d not valid.", param);
        return(INVALIDPARAMETER);
    }

    // Check bounds, update value and send notification or indication if possible.
    if(len <= valMaxLen && len >= valMinLen)
    {
        memcpy(pAttrVal, value, len);
        *pValLen = len; // Update length for read and get.
    }
    else
    {
        Log_error3("Length outside bounds: Len: %d MinLen: %d MaxLen: %d.", len,
                   valMinLen,
                   valMaxLen);
        ret = bleInvalidRange;
    }

    return(ret);
}

/*
 * LedService_GetParameter - Get a LedService parameter.
 *
 *    param - Profile parameter ID
 *    len   - pointer to a variable that contains the maximum length that can be written to *value.
              After the call, this value will contain the actual returned length.
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t LedService_GetParameter(uint8_t param, uint16_t *len, void *value)
{
    bStatus_t ret = SUCCESS;
    switch(param)
    {
    case LS_MODE_ID:
        *len = MIN(*len, ls_ModeValLen);
        memcpy(value, ls_ModeVal, *len);
        Log_info2("GetParameter : %s returning %d bytes", (uintptr_t)"MODE",
                  *len);
        break;

    case LS_LED1_ID:
        *len = MIN(*len, ls_LED1ValLen);
        memcpy(value, ls_LED1Val, *len);
        Log_info2("GetParameter : %s returning %d bytes", (uintptr_t)"LED1",
                  *len);
        break;

    default:
        Log_error1("GetParameter: Parameter #%d not valid.", param);
        ret = INVALIDPARAMETER;
        break;
    }
    return(ret);
}

/*********************************************************************
 * @internal
 * @fn          LED_Service_findCharParamId
 *
 * @brief       Find the logical param id of an attribute in the service's attr table.
 *
 *              Works only for Characteristic Value attributes and
 *              Client Characteristic Configuration Descriptor attributes.
 *
 * @param       pAttr - pointer to attribute
 *
 * @return      uint8_t paramID (ref led_service.h) or 0xFF if not found.
 */
static uint8_t LED_Service_findCharParamId(gattAttribute_t *pAttr)
{
    // Is this a Client Characteristic Configuration Descriptor?
    if(ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID ==
       *(uint16_t *)pAttr->type.uuid)
    {
        return(LED_Service_findCharParamId(pAttr - 1)); // Assume the value attribute precedes CCCD and recurse
    }
    // Is this attribute in "Mode"?
    else if(ATT_BT_UUID_SIZE == pAttr->type.len &&
            !memcmp(pAttr->type.uuid, ls_MODEUUID, pAttr->type.len))
    {
        return(LS_MODE_ID);
    }
    // Is this attribute in "LED1"?
    else if(ATT_UUID_SIZE == pAttr->type.len &&
            !memcmp(pAttr->type.uuid, ls_LED1UUID, pAttr->type.len))
    {
        return(LS_LED1_ID);
    }
    else
    {
        return(0xFF); // Not found. Return invalid.
    }
}

/*********************************************************************
 * @fn          LED_Service_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t LED_Service_ReadAttrCB(uint16_t connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t *pLen,
                                        uint16_t offset,
                                        uint16_t maxLen,
                                        uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint16_t valueLen;
    uint8_t paramID = 0xFF;

    // Find settings for the characteristic to be read.
    paramID = LED_Service_findCharParamId(pAttr);
    switch(paramID)
    {
    case LS_MODE_ID:
        valueLen = ls_ModeValLen;

        Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                  (uintptr_t)"MODE",
                  connHandle,
                  offset,
                  method);
        /* Other considerations for MODE can be inserted here */
        break;

    case LS_LED1_ID:
        valueLen = ls_LED1ValLen;

        Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                  (uintptr_t)"LED1",
                  connHandle,
                  offset,
                  method);
        /* Other considerations for LED1 can be inserted here */
        break;

    default:
        Log_error0("Attribute was not found.");
        return(ATT_ERR_ATTR_NOT_FOUND);
    }
    // Check bounds and return the value
    if(offset > valueLen)   // Prevent malicious ATT ReadBlob offsets.
    {
        Log_error0("An invalid offset was requested.");
        status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
        *pLen = MIN(maxLen, valueLen - offset); // Transmit as much as possible
        memcpy(pValue, pAttr->pValue + offset, *pLen);
    }

    return(status);
}

/*********************************************************************
 * @fn      LED_Service_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t LED_Service_WriteAttrCB(uint16_t connHandle,
                                         gattAttribute_t *pAttr,
                                         uint8_t *pValue, uint16_t len,
                                         uint16_t offset,
                                         uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t paramID = 0xFF;
    uint8_t changeParamID = 0xFF;
    uint16_t writeLenMin;
    uint16_t writeLenMax;
    uint16_t *pValueLenVar;

    // Find settings for the characteristic to be written.
    paramID = LED_Service_findCharParamId(pAttr);
    switch(paramID)
    {
    case LS_MODE_ID:
        writeLenMin = LS_MODE_LEN_MIN;
        writeLenMax = LS_MODE_LEN;
        pValueLenVar = &ls_ModeValLen;

        Log_info5(
            "WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
            (uintptr_t)"MODE",
            connHandle,
            len,
            offset,
            method);
        /* Other considerations for MODE can be inserted here */
        break;

    case LS_LED1_ID:
        writeLenMin = LS_LED1_LEN_MIN;
        writeLenMax = LS_LED1_LEN;
        pValueLenVar = &ls_LED1ValLen;

        Log_info5(
            "WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
            (uintptr_t)"LED1",
            connHandle,
            len,
            offset,
            method);
        /* Other considerations for LED1 can be inserted here */
        break;

    default:
        Log_error0("Attribute was not found.");
        return(ATT_ERR_ATTR_NOT_FOUND);
    }
    // Check whether the length is within bounds.
    if(offset >= writeLenMax)
    {
        Log_error0("An invalid offset was requested.");
        status = ATT_ERR_INVALID_OFFSET;
    }
    else if(offset + len > writeLenMax)
    {
        Log_error0("Invalid value length was received.");
        status = ATT_ERR_INVALID_VALUE_SIZE;
    }
    else if(offset + len < writeLenMin &&
            (method == ATT_EXECUTE_WRITE_REQ || method == ATT_WRITE_REQ))
    {
        // Refuse writes that are lower than minimum.
        // Note: Cannot determine if a Reliable Write (to several chars) is finished, so those will
        //       only be refused if this attribute is the last in the queue (method is execute).
        //       Otherwise, reliable writes are accepted and parsed piecemeal.
        Log_error0("Invalid value length was received.");
        status = ATT_ERR_INVALID_VALUE_SIZE;
    }
    else
    {
        // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application and update length if enough data is written.
        //
        // Note: If reliable writes are used (meaning several attributes are written to using ATT PrepareWrite),
        //       the application will get a callback for every write with an offset + len larger than _LEN_MIN.
        // Note: For Long Writes (ATT Prepare + Execute towards only one attribute) only one callback will be issued,
        //       because the write fragments are concatenated before being sent here.
        if(offset + len >= writeLenMin)
        {
            changeParamID = paramID;
            *pValueLenVar = offset + len; // Update data length.
        }
    }

    // Let the application know something changed (if it did) by using the
    // callback it registered earlier (if it did).
    if(changeParamID != 0xFF)
    {
        if(pAppCBs && pAppCBs->pfnChangeCb)
        {
            pAppCBs->pfnChangeCb(connHandle, paramID, len + offset, pValue); // Call app function from stack task context.
        }
    }
    return(status);
}

#endif // MOTION_COMPILE
