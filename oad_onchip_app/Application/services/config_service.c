/*
 * data_service.c
 *
 *  Created on: 18 êâ³ò. 2020 ð.
 *      Author: Oleh
 */

/*********************************************************************
 * INCLUDES
 */
#if MOTION_COMPILE

#include <string.h>

//#include <xdc/runtime/Log.h> // Comment this in to use xdc.runtime.Log

#include <icall.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include <services/config_service.h>
#include "device_common.h"

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

// Config_Service Service UUID
CONST uint8_t ConfigServiceUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(CONFIG_SERVICE_SERV_UUID), HI_UINT16(CONFIG_SERVICE_SERV_UUID)
};

// Config mode UUID
CONST uint8_t cs_ModeUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(ÑS_MODE_UUID), HI_UINT16(ÑS_MODE_UUID)
};

#if 0
// Config sensitivity UUID
CONST uint8_t cs_SensitivityUUID[ATT_BT_UUID_SIZE] =
{
    BASE128_FROM_UINT16(ÑS_SENSITIVITY_UUID)
};
#endif
// MODE UUID
CONST uint8_t cs_LEDUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(CS_LED_UUID), HI_UINT16(CS_LED_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static ConfigServiceCBs_t *pAppCBs = NULL;
static uint8_t cs_icall_rsp_task_id = INVALID_TASK_ID;

/*********************************************************************
 * Profile Attributes - variables
 */

// Service declaration
static CONST gattAttrType_t DataServiceDecl = { ATT_UUID_SIZE, ConfigServiceUUID };

// Characteristic "Mode" Properties (for declaration)
static uint8_t cs_ModeProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "Mode" Value variable
static uint8_t cs_ModeVal[CS_MODE_LEN] = { 0 };

// Length of data in characteristic "Mode" Value variable
static uint16_t cs_ModeValLen = CS_MODE_LEN;

#if 0
// Characteristic "Sensitivity" Properties (for declaration)
static uint8_t cs_SensitivityProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic "Sensitivity" Value variable
static uint8_t cs_SensitivityVal[CS_SENSITIVITY_LEN] = { MOTION_SENSITIVITY_3M };

// Length of data in characteristic "Sensitivity" Value variable
static uint16_t cs_SensitivityValLen = CS_SENSITIVITY_LEN;
#endif

// Characteristic "LedMODE" Properties (for declaration)
static uint8_t cs_LedProps = GATT_PROP_READ | GATT_PROP_WRITE |
                              GATT_PROP_WRITE_NO_RSP; // TODO: NO_RSP?

// Characteristic "LedMODE" Value variable
static uint8_t cs_LedVal[CS_LED_LEN] = { 0x01 };

// Length of data in characteristic "MODE" Value variable, initialized to minimal size.
static uint16_t cs_LedValLen = CS_LED_LEN_MIN;
/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t Config_ServiceAttrTbl[] =
{
    // Data_Service Service Declaration
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)&DataServiceDecl
    },
        // Mode Characteristic Declaration
        {
            { ATT_BT_UUID_SIZE, characterUUID },
            GATT_PERMIT_READ,
            0,
            &cs_ModeProps
        },
            // Mode Characteristic Value
            {
                { ATT_BT_UUID_SIZE, cs_ModeUUID },
                GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                0,
                cs_ModeVal
            },
#if 0
        // Sensitivity Characteristic Declaration
        {
            { ATT_BT_UUID_SIZE, characterUUID },
            GATT_PERMIT_READ,
            0,
            &cs_SensitivityProps
        },
            // Stream Characteristic Value
            {
                { ATT_UUID_SIZE, cs_SensitivityUUID },
                GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                0,
                cs_SensitivityVal
            },
#endif
        // LedMode Characteristic Declaration
        {
            { ATT_BT_UUID_SIZE, characterUUID },
            GATT_PERMIT_READ,
            0,
            &cs_LedProps
        },
            // Led Mode Characteristic Value
            {
                { ATT_BT_UUID_SIZE, cs_LEDUUID },
                GATT_PERMIT_READ | GATT_PERMIT_WRITE,
                0,
                cs_LedVal
            },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Config_Service_ReadAttrCB(uint16_t connHandle,
                                         gattAttribute_t *pAttr,
                                         uint8_t *pValue,
                                         uint16_t *pLen,
                                         uint16_t offset,
                                         uint16_t maxLen,
                                         uint8_t method);
static bStatus_t Config_Service_WriteAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue,
                                          uint16_t len,
                                          uint16_t offset,
                                          uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t Config_ServiceCBs =
{
    Config_Service_ReadAttrCB, // Read callback function pointer
    Config_Service_WriteAttrCB, // Write callback function pointer
    NULL                     // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * ConfigService_AddService- Initializes the DataService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t ConfigService_AddService(uint8_t rspTaskId)
{
    uint8_t status;

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(Config_ServiceAttrTbl,
                                         GATT_NUM_ATTRS(Config_ServiceAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &Config_ServiceCBs);

    cs_icall_rsp_task_id = rspTaskId;

    return(status);
}

/*
 * ConfigService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t ConfigService_RegisterAppCBs(ConfigServiceCBs_t *appCallbacks)
{
    if(appCallbacks)
    {
        pAppCBs = appCallbacks;
        return(SUCCESS);
    }
    else
    {
        Log_warning0("Null pointer given for app callbacks.");
        return(FAILURE);
    }
}

/*
 * ConfigService_SetParameter - Set a DataService parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t ConfigService_SetParameter(uint8_t param, uint16_t len, void *value)
{
    bStatus_t ret = SUCCESS;
    uint8_t  *pAttrVal;
    uint16_t *pValLen;
    uint16_t valLen;
    uint8_t sendNotiInd = FALSE;
    gattCharCfg_t *attrConfig = NULL;
    uint8_t needAuth;

    switch(param)
    {
    case CS_MODE_ID:
        pAttrVal = cs_ModeVal;
        pValLen = &cs_ModeValLen;
        valLen = CS_MODE_LEN;
        needAuth = FALSE;  // Change if authenticated link is required for sending.
        break;
#if 0
    case CS_SENSITIVITY_ID:
        pAttrVal = cs_SensitivityVal;
        pValLen = &cs_SensitivityValLen;
        valLen = CS_SENSITIVITY_LEN;
        needAuth = FALSE;  // Change if authenticated link is required for sending.
        Log_info2("SetParameter : %s len: %d", (uintptr_t)"Mode", len);
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
            // Try to send notification.
            GATTServApp_ProcessCharCfg(attrConfig, pAttrVal, needAuth,
                                       Config_ServiceAttrTbl,
                                       GATT_NUM_ATTRS(
                                           Config_ServiceAttrTbl),
                                       cs_icall_rsp_task_id,
                                       Config_Service_ReadAttrCB);
        }
    }
    else
    {
        Log_error2("Length mismatch: Len: %d, expected len: %d.", len, valLen);
        ret = bleInvalidRange;
    }

    return(ret);
}

/*
 * ConfigService_GetParameter - Get a DataService parameter.
 *
 *    param - Profile parameter ID
 *    len   - pointer to a variable that contains the maximum length that can be written to *value.
              After the call, this value will contain the actual returned length.
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t ConfigService_GetParameter(uint8_t param, uint16_t *len, void *value)
{
    bStatus_t ret = SUCCESS;
    switch(param)
    {
    case CS_MODE_ID:
        memcpy(value, cs_ModeVal, *len);
        break;
#if 0
    case CS_SENSITIVITY_ID:
        memcpy(value, cs_SensitivityVal, *len);
        Log_info2("GetParameter : %s returning %d", (uintptr_t)"Sensitivity",
                  *(uint8_t*)value);
        break;
#endif
    case CS_LED_ID:
        *len = MIN(*len, cs_LedValLen);
        memcpy(value, cs_LedVal, *len);
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
 * @fn          Config_Service_findCharParamId
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
static uint8_t Config_Service_findCharParamId(gattAttribute_t *pAttr)
{
    // Is this a Client Characteristic Configuration Descriptor?
    if(ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID ==
       *(uint16_t *)pAttr->type.uuid)
    {
        return(Config_Service_findCharParamId(pAttr - 1)); // Assume the value attribute precedes CCCD and recurse
    }
    // Is this attribute in "Mode"?
    else if(ATT_BT_UUID_SIZE == pAttr->type.len &&
            !memcmp(pAttr->type.uuid, cs_ModeUUID, pAttr->type.len))
    {
        return(CS_MODE_ID);
    }
#if 0
    // Is this attribute in "Sensitivity"?
    else if(ATT_UUID_SIZE == pAttr->type.len &&
            !memcmp(pAttr->type.uuid, cs_SensitivityUUID, pAttr->type.len))
    {
        return(CS_SENSITIVITY_ID);
    }
#endif
    // Is this attribute in "Led Mode"?
    else if(ATT_BT_UUID_SIZE == pAttr->type.len &&
            !memcmp(pAttr->type.uuid, cs_LEDUUID, pAttr->type.len))
    {
        return(CS_LED_ID);
    }
    else
    {
        return(0xFF); // Not found. Return invalid.
    }
}

/*********************************************************************
 * @fn          Config_Service_ReadAttrCB
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
static bStatus_t Config_Service_ReadAttrCB(uint16_t connHandle,
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
    paramID = Config_Service_findCharParamId(pAttr);
    switch(paramID)
    {
    case CS_MODE_ID:
        valueLen = cs_ModeValLen;
        /* Other considerations for Stream can be inserted here */
        break;

    case CS_LED_ID:
        valueLen = cs_LedValLen;
        /* Other considerations for MODE can be inserted here */
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
 * @fn      Config_Service_WriteAttrCB
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
static bStatus_t Config_Service_WriteAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t len,
                                          uint16_t offset,
                                          uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t paramID = 0xFF;
    uint8_t changeParamID = 0xFF;
    uint16_t writeLen;
    uint16_t *pValueLenVar;

    // See if request is regarding a Client Characterisic Configuration
    if(ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID ==
       *(uint16_t *)pAttr->type.uuid)
    {
        // Allow notification and indication, but do not check if really allowed per CCCD.
        status = GATTServApp_ProcessCCCWriteReq(
            connHandle, pAttr, pValue, len,
            offset,
            GATT_CLIENT_CFG_NOTIFY |
            GATT_CLIENT_CFG_INDICATE);
        if(SUCCESS == status && pAppCBs && pAppCBs->pfnCfgChangeCb)
        {
            pAppCBs->pfnCfgChangeCb(connHandle,
                                    Config_Service_findCharParamId(
                                        pAttr), len, pValue);
        }

        return(status);
    }

    // Find settings for the characteristic to be written.
    paramID = Config_Service_findCharParamId(pAttr);
    switch(paramID)
    {
    case CS_MODE_ID:
        writeLen = CS_MODE_LEN;
        pValueLenVar = &cs_ModeValLen;

        Log_info5(
            "WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
            (uintptr_t)"Mode",
            connHandle,
            len,
            offset,
            method);
        /* Other considerations for Stream can be inserted here */
        break;

#if 0
    case CS_SENSITIVITY_ID:
        writeLen = CS_SENSITIVITY_LEN;
        pValueLenVar = &cs_SensitivityValLen;

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
    case CS_LED_ID:
         writeLen = CS_LED_LEN;
         pValueLenVar = &cs_LedValLen;

         Log_info5(
             "WriteAttrCB : %s connHandle(%d) len(%d) offset(%d) method(0x%02x)",
             (uintptr_t)"LED",
             connHandle,
             len,
             offset,
             method);
         /* Other considerations for MODE can be inserted here */
         break;
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
