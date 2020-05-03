/*
 * tamper_service.c
 *
 *  Created on: 2 трав. 2020 р.
 *      Author: Oleh
 */



/*********************************************************************
 * INCLUDES
 */
#include <string.h>

//#include <xdc/runtime/Log.h> // Comment this in to use xdc.runtime.Log
#include <uartlog/UartLog.h>  // Comment out if using xdc Log

#include <icall.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include <services/tamper_service.h>

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

// Tamper_Service Service UUID
CONST uint8_t TamperServiceUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(TAMPER_SERVICE_SERV_UUID), HI_UINT16(TAMPER_SERVICE_SERV_UUID)
};

// TAMPER UUID
CONST uint8_t ts_TamperStateUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(TS_STATE_UUID), HI_UINT16(TS_STATE_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static TamperServiceCBs_t *pAppCBs = NULL;
static uint8_t ts_icall_rsp_task_id = INVALID_TASK_ID;

/*********************************************************************
 * Profile Attributes - variables
 */

// Service declaration
static CONST gattAttrType_t TamperServiceDecl =
{ ATT_BT_UUID_SIZE, TamperServiceUUID };

// Characteristic "TAMPER" Properties (for declaration)
static uint8_t ts_TamperProps = GATT_PROP_NOTIFY | GATT_PROP_READ
        | GATT_PROP_INDICATE;

// Characteristic "TAMPER" Value variable
static uint8_t ts_TamperStateVal[TS_STATE_LEN] = { TAMPER_STATE_PRESS };

// Length of data in characteristic "TAMPER" Value variable, initialized to minimal size.
static uint16_t ts_TamperStateValLen = TS_STATE_LEN;

// Characteristic "TAMPER" Client Characteristic Configuration Descriptor
static gattCharCfg_t *ts_TamperStateConfig;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t Tamper_ServiceAttrTbl[] =
{
    // Tamper_Service Service Declaration
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)&TamperServiceDecl
    },
        // TAMPER Characteristic Declaration
        {
            { ATT_BT_UUID_SIZE, characterUUID },
            GATT_PERMIT_READ,
            0,
            &ts_TamperProps
        },
            // TAMPER Characteristic Value
            {
                { ATT_BT_UUID_SIZE, ts_TamperStateUUID },
                GATT_PERMIT_READ,
                0,
                ts_TamperStateVal
            },
            // TAMPER CCCD
            {
                { ATT_BT_UUID_SIZE, clientCharCfgUUID },
                GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                0,
                (uint8_t *)&ts_TamperStateConfig
            },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Tamper_Service_ReadAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue,
                                           uint16_t *pLen,
                                           uint16_t offset,
                                           uint16_t maxLen,
                                           uint8_t method);
static bStatus_t Tamper_Service_WriteAttrCB(uint16_t connHandle,
                                            gattAttribute_t *pAttr,
                                            uint8_t *pValue,
                                            uint16_t len,
                                            uint16_t offset,
                                            uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t Tamper_ServiceCBs =
{
    Tamper_Service_ReadAttrCB, // Read callback function pointer
    Tamper_Service_WriteAttrCB, // Write callback function pointer
    NULL                     // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * TamperService_AddService- Initializes the TamperService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t TamperService_AddService(uint8_t rspTaskId)
{
    uint8_t status;

    // Allocate Client Characteristic Configuration table
    ts_TamperStateConfig = (gattCharCfg_t *)ICall_malloc(
        sizeof(gattCharCfg_t) * linkDBNumConns);
    if(ts_TamperStateConfig == NULL)
    {
        return(bleMemAllocError);
    }

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg(CONNHANDLE_INVALID, ts_TamperStateConfig);

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(Tamper_ServiceAttrTbl,
                                         GATT_NUM_ATTRS(Tamper_ServiceAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &Tamper_ServiceCBs);
    Log_info1("Registered service, %d attributes",
              GATT_NUM_ATTRS(Tamper_ServiceAttrTbl));
    ts_icall_rsp_task_id = rspTaskId;

    return(status);
}

/*
 * TamperService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t TamperService_RegisterAppCBs(TamperServiceCBs_t *appCallbacks)
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
 * TamperService_SetParameter - Set a TamperService parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t TamperService_SetParameter(uint8_t param, uint16_t len, void *value)
{
    bStatus_t ret = SUCCESS;
    uint8_t  *pAttrVal;
    uint16_t *pValLen;
    uint8_t sendNotiInd = FALSE;
    gattCharCfg_t *attrConfig;
    uint8_t needAuth;

    switch(param)
    {
    case TS_STATE_ID:
        pAttrVal = ts_TamperStateVal;
        pValLen = &ts_TamperStateValLen;
        sendNotiInd = TRUE;
        attrConfig = ts_TamperStateConfig;
        attrConfig[0].connHandle = 0;
        needAuth = FALSE;  // Change if authenticated link is required for sending.
        Log_info3("%s: Set %s len: %d", (uintptr_t )__func__,
                  (uintptr_t )"STATE", len);
        break;

    default:
        Log_error1("SetParameter: Parameter #%d not valid.", param);
        return(INVALIDPARAMETER);
    }

    // Check bounds, update value and send notification or indication if possible.
    if(len == TS_STATE_LEN)
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
                                       Tamper_ServiceAttrTbl,
                                       GATT_NUM_ATTRS(
                                           Tamper_ServiceAttrTbl),
                                       ts_icall_rsp_task_id,
                                       Tamper_Service_ReadAttrCB);
        }
    }
    else
    {
        Log_error2("Length outside bounds: Len: %d RequiredLen: %d", len,
                   TS_STATE_LEN);
        ret = bleInvalidRange;
    }

    return(ret);
}

/*
 * TamperService_GetParameter - Get a TamperService parameter.
 *
 *    param - Profile parameter ID
 *    len   - pointer to a variable that contains the maximum length that can be written to *value.
              After the call, this value will contain the actual returned length.
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t TamperService_GetParameter(uint8_t param, uint16_t *len, void *value)
{
    bStatus_t ret = SUCCESS;
    switch(param)
    {
    default:
        Log_error1("GetParameter: Parameter #%d not valid.", param);
        ret = INVALIDPARAMETER;
        break;
    }
    return(ret);
}

/*********************************************************************
 * @internal
 * @fn          Tamper_Service_findCharParamId
 *
 * @brief       Find the logical param id of an attribute in the service's attr table.
 *
 *              Works only for Characteristic Value attributes and
 *              Client Characteristic Configuration Descriptor attributes.
 *
 * @param       pAttr - pointer to attribute
 *
 * @return      uint8_t paramID (ref Tamper_service.h) or 0xFF if not found.
 */
static uint8_t Tamper_Service_findCharParamId(gattAttribute_t *pAttr)
{
    // Is this a Client Characteristic Configuration Descriptor?
    if(ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID ==
       *(uint16_t *)pAttr->type.uuid)
    {
        return(Tamper_Service_findCharParamId(pAttr - 1)); // Assume the value attribute precedes CCCD and recurse
    }
    // Is this attribute in "STATE"?
    else if(ATT_BT_UUID_SIZE == pAttr->type.len &&
            !memcmp(pAttr->type.uuid, ts_TamperStateUUID, pAttr->type.len))
    {
        return(TS_STATE_ID);
    }
    else
    {
        return(0xFF); // Not found. Return invalid.
    }
}

/*********************************************************************
 * @fn          Tamper_Service_ReadAttrCB
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
static bStatus_t Tamper_Service_ReadAttrCB(uint16_t connHandle,
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
    paramID = Tamper_Service_findCharParamId(pAttr);
    switch(paramID)
    {
    case TS_STATE_ID:
        valueLen = ts_TamperStateValLen;

        Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                  (uintptr_t)"STATE",
                  connHandle,
                  offset,
                  method);
        /* Other considerations for Tamper STATE can be inserted here */
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
 * @fn      Tamper_Service_WriteAttrCB
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
static bStatus_t Tamper_Service_WriteAttrCB(uint16_t connHandle,
                                            gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len,
                                            uint16_t offset,
                                            uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t paramID = 0xFF;

    // See if request is regarding a Client Characterisic Configuration
    if(ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID ==
       *(uint16_t *)pAttr->type.uuid)
    {
        Log_info3("WriteAttrCB (CCCD): param: %d connHandle: %d %s",
                  Tamper_Service_findCharParamId(pAttr),
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
                                    Tamper_Service_findCharParamId(
                                        pAttr), len, pValue);
        }

        return(status);
    }

    // Find settings for the characteristic to be written.
    paramID = Tamper_Service_findCharParamId(pAttr);
    switch(paramID)
    {
    default:
        Log_error0("Attribute was not found.");
        return(ATT_ERR_ATTR_NOT_FOUND);
    }
}
