/*
 * data_service.c
 *
 *  Created on: 18 квіт. 2020 р.
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

#include <services/data_service.h>

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

// Data_Service Service UUID
CONST uint8_t DataServiceUUID[ATT_UUID_SIZE] =
{
    BASE128_FROM_UINT16(DATA_SERVICE_SERV_UUID)
};

CONST uint8_t ds_StateUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(DS_STATE_UUID), HI_UINT16(DS_STATE_UUID)
};
#if 0
// String UUID
CONST uint8_t ds_StringUUID[ATT_UUID_SIZE] =
{
    BASE128_FROM_UINT16(DS_STRING_UUID)
};

// Stream UUID
CONST uint8_t ds_StreamUUID[ATT_UUID_SIZE] =
{
    BASE128_FROM_UINT16(DS_STREAM_UUID)
};
#endif
/*********************************************************************
 * LOCAL VARIABLES
 */

static DataServiceCBs_t *pAppCBs = NULL;
static uint8_t ds_icall_rsp_task_id = INVALID_TASK_ID;

/*********************************************************************
 * Profile Attributes - variables
 */

// Service declaration
static CONST gattAttrType_t DataServiceDecl = { ATT_UUID_SIZE, DataServiceUUID };

// Characteristic "State" Properties (for declaration)
static uint8_t ds_StateProps = GATT_PROP_READ | GATT_PROP_NOTIFY
        | GATT_PROP_INDICATE;

// Characteristic "State" Value variable
static uint8_t ds_StateVal[DS_STATE_LEN] = {0};

// Length of data in characteristic "State" Value variable.
static uint16_t ds_StateValLen = DS_STATE_LEN;

// Characteristic "State" Client Characteristic Configuration Descriptor
static gattCharCfg_t *ds_StateConfig;

#if 0
// Characteristic "String" Properties (for declaration)
static uint8_t ds_StringProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "String" Value variable
static uint8_t ds_StringVal[DS_STRING_LEN] = {0};

// Length of data in characteristic "String" Value variable, initialized to minimal size.
static uint16_t ds_StringValLen = DS_STRING_LEN_MIN;

// Characteristic "Stream" Properties (for declaration)
static uint8_t ds_StreamProps = GATT_PROP_NOTIFY | GATT_PROP_WRITE_NO_RSP;

// Characteristic "Stream" Value variable
static uint8_t ds_StreamVal[DS_STREAM_LEN] = {0};

// Length of data in characteristic "Stream" Value variable, initialized to minimal size.
static uint16_t ds_StreamValLen = DS_STREAM_LEN_MIN;

// Characteristic "Stream" Client Characteristic Configuration Descriptor
static gattCharCfg_t *ds_StreamConfig;
#endif
/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t Data_ServiceAttrTbl[] =
{
    // Data_Service Service Declaration
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)&DataServiceDecl
    },

    // String Characteristic Declaration
        {
            { ATT_BT_UUID_SIZE, characterUUID },
            GATT_PERMIT_READ,
            0,
            &ds_StateProps
        },
            // State Characteristic Value
            {
                { ATT_BT_UUID_SIZE, ds_StateUUID },
                GATT_PERMIT_READ,
                0,
                ds_StateVal
            },
            // State CCCD
            {
                { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                0,
                (uint8_t *)&ds_StateConfig
            },
#if 0
        // String Characteristic Declaration
        {
            { ATT_BT_UUID_SIZE, characterUUID },
            GATT_PERMIT_READ,
            0,
            &ds_StringProps
        },
            // String Characteristic Value
            {
                { ATT_UUID_SIZE, ds_StringUUID },
                GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                0,
                ds_StringVal
            },
        // Stream Characteristic Declaration
        {
            { ATT_BT_UUID_SIZE, characterUUID },
            GATT_PERMIT_READ,
            0,
            &ds_StreamProps
        },
            // Stream Characteristic Value
            {
                { ATT_UUID_SIZE, ds_StreamUUID },
                GATT_PERMIT_WRITE,
                0,
                ds_StreamVal
            },
            // Stream CCCD
            {
                { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                0,
                (uint8_t *)&ds_StreamConfig
            },
#endif
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Data_Service_ReadAttrCB(uint16_t connHandle,
                                         gattAttribute_t *pAttr,
                                         uint8_t *pValue,
                                         uint16_t *pLen,
                                         uint16_t offset,
                                         uint16_t maxLen,
                                         uint8_t method);
static bStatus_t Data_Service_WriteAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue,
                                          uint16_t len,
                                          uint16_t offset,
                                          uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t Data_ServiceCBs =
{
    Data_Service_ReadAttrCB, // Read callback function pointer
    Data_Service_WriteAttrCB, // Write callback function pointer
    NULL                     // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * DataService_AddService- Initializes the DataService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t DataService_AddService(uint8_t rspTaskId)
{
    uint8_t status;

    // Allocate Client Characteristic Configuration table
    ds_StateConfig = (gattCharCfg_t *)ICall_malloc(
        sizeof(gattCharCfg_t) * linkDBNumConns);
    if(ds_StateConfig == NULL)
    {
        return(bleMemAllocError);
    }

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg(CONNHANDLE_INVALID, ds_StateConfig);
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(Data_ServiceAttrTbl,
                                         GATT_NUM_ATTRS(Data_ServiceAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &Data_ServiceCBs);
    Log_info1("Registered service, %d attributes",
              GATT_NUM_ATTRS(Data_ServiceAttrTbl));
    ds_icall_rsp_task_id = rspTaskId;

    return(status);
}

/*
 * DataService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t DataService_RegisterAppCBs(DataServiceCBs_t *appCallbacks)
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
 * DataService_SetParameter - Set a DataService parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t DataService_SetParameter(uint8_t param, uint16_t len, void *value)
{
    bStatus_t ret = SUCCESS;
    uint8_t  *pAttrVal;
    uint16_t *pValLen;
    uint16_t valLen;
    uint8_t sendNotiInd = FALSE;
    gattCharCfg_t *attrConfig;
    uint8_t needAuth;

    switch(param)
    {
    case DS_STATE_ID:
        pAttrVal = ds_StateVal;
        pValLen = &ds_StateValLen;
        valLen = DS_STATE_LEN;
        sendNotiInd = TRUE;
        attrConfig = ds_StateConfig;
        needAuth = FALSE;  // Change if authenticated link is required for sending.
        Log_info2("SetParameter : %s len: %d", (uintptr_t)"State", len);
        break;

#if 0
    case DS_STRING_ID:
        pAttrVal = ds_StringVal;
        pValLen = &ds_StringValLen;
        valMinLen = DS_STRING_LEN_MIN;
        valMaxLen = DS_STRING_LEN;
        Log_info2("SetParameter : %s len: %d", (uintptr_t)"String", len);
        break;
    case DS_STREAM_ID:
        pAttrVal = ds_StreamVal;
        pValLen = &ds_StreamValLen;
        valMinLen = DS_STREAM_LEN_MIN;
        valMaxLen = DS_STREAM_LEN;
        sendNotiInd = TRUE;
        attrConfig = ds_StreamConfig;
        needAuth = FALSE;  // Change if authenticated link is required for sending.
        Log_info2("SetParameter : %s len: %d", (uintptr_t)"Stream", len);
        break;
#endif
    default:
        Log_error1("SetParameter: Parameter #%d not valid.", param);
        return(INVALIDPARAMETER);
    }

    // Check bounds, update value and send notification or indication if possible.
    if(len == valLen)
    {
        memcpy(pAttrVal, value, len);
        *pValLen = len; // Update length for read and get.

        if(sendNotiInd)
        {
            Log_info2("Trying to send noti/ind: connHandle 0x%x, %s",
                      attrConfig[0].connHandle,
                      (uintptr_t)((attrConfig[0].value ==
                                   0) ? "\x1b[33mNoti/ind disabled\x1b[0m" :
                                  (attrConfig[0].value ==
                                   1) ? "Notification enabled" :
                                  "Indication enabled"));
            // Try to send notification.
            GATTServApp_ProcessCharCfg(attrConfig, pAttrVal, needAuth,
                                       Data_ServiceAttrTbl,
                                       GATT_NUM_ATTRS(
                                           Data_ServiceAttrTbl),
                                       ds_icall_rsp_task_id,
                                       Data_Service_ReadAttrCB);
        }
    }
    else
    {
        Log_error2("Length outside bounds: Len: %d, written Len: %d.", len,
                   valLen);
        ret = bleInvalidRange;
    }

    return(ret);
}

/*
 * DataService_GetParameter - Get a DataService parameter.
 *
 *    param - Profile parameter ID
 *    len   - pointer to a variable that contains the maximum length that can be written to *value.
              After the call, this value will contain the actual returned length.
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t DataService_GetParameter(uint8_t param, uint16_t *len, void *value)
{
    bStatus_t ret = SUCCESS;
    switch(param)
    {
    case DS_STATE_ID:
        memcpy(value, ds_StateVal, *len);
        Log_info2("GetParameter : %s returning %d bytes", (uintptr_t)"State",
                  *len);
        break;
#if 0
    case DS_STRING_ID:
        *len = MIN(*len, ds_StringValLen);
        memcpy(value, ds_StringVal, *len);
        Log_info2("GetParameter : %s returning %d bytes", (uintptr_t)"String",
                  *len);
        break;

    case DS_STREAM_ID:
        *len = MIN(*len, ds_StreamValLen);
        memcpy(value, ds_StreamVal, *len);
        Log_info2("GetParameter : %s returning %d bytes", (uintptr_t)"Stream",
                  *len);
        break;
#endif
    default:
        Log_error1("GetParameter: Parameter #%d not valid.", param);
        ret = INVALIDPARAMETER;
        break;
    }
    return(ret);
}

/*********************************************************************
 * @internal
 * @fn          Data_Service_findCharParamId
 *
 * @brief       Find the logical param id of an attribute in the service's attr table.
 *
 *              Works only for Characteristic Value attributes and
 *              Client Characteristic Configuration Descriptor attributes.
 *
 * @param       pAttr - pointer to attribute
 *
 * @return      uint8_t paramID (ref data_service.h) or 0xFF if not found.
 */
static uint8_t Data_Service_findCharParamId(gattAttribute_t *pAttr)
{
    // Is this a Client Characteristic Configuration Descriptor?
    if(ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID ==
       *(uint16_t *)pAttr->type.uuid)
    {
        return(Data_Service_findCharParamId(pAttr - 1)); // Assume the value attribute precedes CCCD and recurse
    }
    // Is this attribute in "State"?
    else if(ATT_BT_UUID_SIZE == pAttr->type.len &&
            !memcmp(pAttr->type.uuid, ds_StateUUID, pAttr->type.len))
    {
        return(DS_STATE_ID);
    }
#if 0
    // Is this attribute in "String"?
    else if(ATT_UUID_SIZE == pAttr->type.len &&
            !memcmp(pAttr->type.uuid, ds_StringUUID, pAttr->type.len))
    {
        return(DS_STRING_ID);
    }
    // Is this attribute in "Stream"?
    else if(ATT_UUID_SIZE == pAttr->type.len &&
            !memcmp(pAttr->type.uuid, ds_StreamUUID, pAttr->type.len))
    {
        return(DS_STREAM_ID);
    }
#endif
    else
    {
        return(0xFF); // Not found. Return invalid.
    }
}

/*********************************************************************
 * @fn          Data_Service_ReadAttrCB
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
static bStatus_t Data_Service_ReadAttrCB(uint16_t connHandle,
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
    paramID = Data_Service_findCharParamId(pAttr);
    switch(paramID)
    {
    case DS_STATE_ID:
        valueLen = ds_StateValLen;

        Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                  (uintptr_t)"State",
                  connHandle,
                  offset,
                  method);
        /* Other considerations for String can be inserted here */
        break;

    #if 0
    case DS_STRING_ID:
        valueLen = ds_StringValLen;

        Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                  (uintptr_t)"String",
                  connHandle,
                  offset,
                  method);
        /* Other considerations for String can be inserted here */
        break;
    case DS_STREAM_ID:
        valueLen = ds_StreamValLen;

        Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                  (uintptr_t)"Stream",
                  connHandle,
                  offset,
                  method);
        /* Other considerations for Stream can be inserted here */
        break;
#endif

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
 * @fn      Data_Service_WriteAttrCB
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
static bStatus_t Data_Service_WriteAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t len,
                                          uint16_t offset,
                                          uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t paramID = 0xFF;
    uint8_t changeParamID = 0xFF;
#if 0
    uint16_t writeLenMin;
    uint16_t writeLenMax;
#endif
    uint16_t writeLen;
    uint16_t *pValueLenVar;

    // See if request is regarding a Client Characterisic Configuration
    if(ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID ==
       *(uint16_t *)pAttr->type.uuid)
    {
        Log_info3("WriteAttrCB (CCCD): param: %d connHandle: %d %s",
                  Data_Service_findCharParamId(pAttr),
                  connHandle,
                  (uintptr_t)(method ==
                              GATT_LOCAL_WRITE ? "- restoring bonded state" :
                              "- OTA write"));

        // Allow notification and indication, but do not check if really allowed per CCCD.
        status = GATTServApp_ProcessCCCWriteReq(
            connHandle, pAttr, pValue, len,
            offset,
            GATT_CLIENT_CFG_NOTIFY |
            GATT_CLIENT_CFG_INDICATE);
        if(SUCCESS == status && pAppCBs && pAppCBs->pfnCfgChangeCb)
        {
            pAppCBs->pfnCfgChangeCb(connHandle,
                                    Data_Service_findCharParamId(
                                        pAttr), len, pValue);
        }

        return(status);
    }

    // Find settings for the characteristic to be written.
    paramID = Data_Service_findCharParamId(pAttr);
    switch(paramID)
    {
    case DS_STATE_ID:
        writeLen = DS_STATE_LEN;
        pValueLenVar = &ds_StateValLen;

        Log_info5(
            "WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
            (uintptr_t)"State",
            connHandle,
            len,
            offset,
            method);
        /* Other considerations for String can be inserted here */
        break;
#if 0
    case DS_STREAM_ID:
        writeLenMin = DS_STREAM_LEN_MIN;
        writeLenMax = DS_STREAM_LEN;
        pValueLenVar = &ds_StreamValLen;

        Log_info5(
            "WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
            (uintptr_t)"Stream",
            connHandle,
            len,
            offset,
            method);
        /* Other considerations for Stream can be inserted here */
        break;
#endif
    default:
        Log_error0("Attribute was not found.");
        return(ATT_ERR_ATTR_NOT_FOUND);
    }
    // Check whether the length is within bounds.
    if(offset >= writeLen)
    {
        Log_error0("An invalid offset was requested.");
        status = ATT_ERR_INVALID_OFFSET;
    }
    else if(offset + len > writeLen)
    {
        Log_error0("Invalid value length was received.");
        status = ATT_ERR_INVALID_VALUE_SIZE;
    }
    else if(offset + len < writeLen &&
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
        if(offset + len >= writeLen)
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
