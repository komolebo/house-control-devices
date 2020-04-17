/******************************************************************************

 @file  devinfoservice.c

 @brief This file contains the Device Information service.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2012-2020, Texas Instruments Incorporated
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

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "devinfoservice.h"
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
// Device information service
CONST uint8 devInfoServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DEVINFO_SERV_UUID), HI_UINT16(DEVINFO_SERV_UUID)
};


// Serial Number String
CONST uint8 devInfoSerialNumberUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SERIAL_NUMBER_UUID), HI_UINT16(SERIAL_NUMBER_UUID)
};
// Software Revision String
CONST uint8 devInfoSoftwareRevUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SOFTWARE_REV_UUID), HI_UINT16(SOFTWARE_REV_UUID)
};
// Device Type
CONST uint8 devInfoTypeUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DEVICE_TYPE_UUID), HI_UINT16(DEVICE_TYPE_UUID)
};
#ifdef PORTING_COMPLETED
// System ID
CONST uint8 devInfoSystemIdUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SYSTEM_ID_UUID), HI_UINT16(SYSTEM_ID_UUID)
};

// Model Number String
CONST uint8 devInfoModelNumberUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MODEL_NUMBER_UUID), HI_UINT16(MODEL_NUMBER_UUID)
};
// Firmware Revision String
CONST uint8 devInfoFirmwareRevUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(FIRMWARE_REV_UUID), HI_UINT16(FIRMWARE_REV_UUID)
};

// Hardware Revision String
CONST uint8 devInfoHardwareRevUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(HARDWARE_REV_UUID), HI_UINT16(HARDWARE_REV_UUID)
};
#endif


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

extern void* memcpy(void *dest, const void *src, size_t len);

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * Profile Attributes - variables
 */

// Device Information Service attribute
static CONST gattAttrType_t devInfoService = { ATT_BT_UUID_SIZE, devInfoServUUID };

// Serial Number String characteristic
static uint8 devInfoSerialNumberProps = GATT_PROP_READ;
static uint8 devInfoSerialNumber[DEVINFO_STR_ATTR_LEN+1] = "Serial Number";

// Software Revision String characteristic
static uint8 devInfoSoftwareRevProps = GATT_PROP_READ;
static uint8 devInfoSoftwareRev[DEVINFO_STR_ATTR_LEN+1] = "Software Revision";

// Serial Number String characteristic
static uint8 devInfoTypeProps = GATT_PROP_READ;
static uint8 devInfoType[DEVINFO_STR_ATTR_LEN+1] = "Base Type";
#if PORTING_COMPLETED
// System ID characteristic
static uint8 devInfoSystemIdProps = GATT_PROP_READ;
static uint8 devInfoSystemId[DEVINFO_SYSTEM_ID_LEN] = {0, 0, 0, 0, 0, 0, 0, 0};

// Model Number String characteristic
static uint8 devInfoModelNumberProps = GATT_PROP_READ;
static uint8 devInfoModelNumber[DEVINFO_STR_ATTR_LEN+1] = "Model Number";

// Firmware Revision String characteristic
static uint8 devInfoFirmwareRevProps = GATT_PROP_READ;
static uint8 devInfoFirmwareRev[DEVINFO_STR_ATTR_LEN+1] = "Firmware Revision";

// Hardware Revision String characteristic
static uint8 devInfoHardwareRevProps = GATT_PROP_READ;
static uint8 devInfoHardwareRev[DEVINFO_STR_ATTR_LEN+1] = "Hardware Revision";
#endif

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t devInfoAttrTbl[] =
{
  // Device Information Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&devInfoService                  /* pValue */
  },

    // Serial Number String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoSerialNumberProps
    },

      // Serial Number Value
      {
        { ATT_BT_UUID_SIZE, devInfoSerialNumberUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoSerialNumber
      },

    // Device Type Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoTypeProps
    },

      // Device Type Value
      {
        { ATT_BT_UUID_SIZE, devInfoTypeUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoType
      },

#if PORTING_COMPLETED
    // System ID Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoSystemIdProps
    },

      // System ID Value
      {
        { ATT_BT_UUID_SIZE, devInfoSystemIdUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoSystemId
      },

    // Model Number String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoModelNumberProps
    },

      // Model Number Value
      {
        { ATT_BT_UUID_SIZE, devInfoModelNumberUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoModelNumber
      },

      // Firmware Revision String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoFirmwareRevProps
    },

      // Firmware Revision Value
      {
        { ATT_BT_UUID_SIZE, devInfoFirmwareRevUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoFirmwareRev
      },

    // Hardware Revision String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoHardwareRevProps
    },

      // Hardware Revision Value
      {
        { ATT_BT_UUID_SIZE, devInfoHardwareRevUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoHardwareRev
      },
#endif

    // Software Revision String Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &devInfoSoftwareRevProps
    },

      // Software Revision Value
      {
        { ATT_BT_UUID_SIZE, devInfoSoftwareRevUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *) devInfoSoftwareRev
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t devInfo_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint16 *pLen, uint16 offset,
                                     uint16 maxLen, uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Device Info Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t devInfoCBs =
{
  devInfo_ReadAttrCB, // Read callback function pointer
  NULL,               // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      DevInfo_AddService
 *
 * @brief   Initializes the Device Information service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t DevInfo_AddService( void )
{
  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService( devInfoAttrTbl,
                                      GATT_NUM_ATTRS( devInfoAttrTbl ),
                                      GATT_MAX_ENCRYPT_KEY_SIZE,
                                      &devInfoCBs );
}

/*********************************************************************
 * @fn      DevInfo_SetParameter
 *
 * @brief   Set a Device Information parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t DevInfo_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case DEVINFO_SERIAL_NUMBER:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoSerialNumber, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoSerialNumber, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_SOFTWARE_REV:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoSoftwareRev, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoSoftwareRev, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }

    case DEVINFO_TYPE:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoType, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoType, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }

#ifdef PORTING_COMPLETED
     case DEVINFO_SYSTEM_ID:
      // verify length
      if (len == sizeof(devInfoSystemId))
      {
        memcpy(devInfoSystemId, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_MODEL_NUMBER:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoModelNumber, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoModelNumber, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_FIRMWARE_REV:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoFirmwareRev, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoFirmwareRev, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case DEVINFO_HARDWARE_REV:
      // verify length, leave room for null-terminate char
      if (len <= DEVINFO_STR_ATTR_LEN)
      {
        memset(devInfoHardwareRev, 0, DEVINFO_STR_ATTR_LEN+1);
        memcpy(devInfoHardwareRev, value, len);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
#endif

      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      DevInfo_GetParameter
 *
 * @brief   Get a Device Information parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t DevInfo_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case DEVINFO_SERIAL_NUMBER:
      memcpy(value, devInfoSerialNumber, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_SOFTWARE_REV:
      memcpy(value, devInfoSoftwareRev, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_TYPE:
      memcpy(value, devInfoType, DEVINFO_STR_ATTR_LEN);
      break;

#ifdef PORTING_COMPLETED
  case DEVINFO_SYSTEM_ID:
      memcpy(value, devInfoSystemId, sizeof(devInfoSystemId));
      break;

    case DEVINFO_MODEL_NUMBER:
      memcpy(value, devInfoModelNumber, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_FIRMWARE_REV:
      memcpy(value, devInfoFirmwareRev, DEVINFO_STR_ATTR_LEN);
      break;

    case DEVINFO_HARDWARE_REV:
      memcpy(value, devInfoHardwareRev, DEVINFO_STR_ATTR_LEN);
      break;
#endif


    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          devInfo_ReadAttrCB
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
static bStatus_t devInfo_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                     uint8 *pValue, uint16 *pLen, uint16 offset,
                                     uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

  // If the value offset of the Read Blob Request is greater than the
  // length of the attribute value, an Error Response shall be sent with
  // the error code Invalid Offset.
  switch (uuid)
  {
#ifdef PORTING_COMPLETED
    case SYSTEM_ID_UUID:
      // verify offset
      if (offset > sizeof(devInfoSystemId))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length
        *pLen = MIN(maxLen, (sizeof(devInfoSystemId) - offset));

        // copy data
        memcpy(pValue, &devInfoSystemId[offset], *pLen);
      }
      break;
#endif

#ifdef PORTING_COMPLETED
    case MODEL_NUMBER_UUID:
    case FIRMWARE_REV_UUID:
    case HARDWARE_REV_UUID:
    case MANUFACTURER_NAME_UUID:
#endif
    case SERIAL_NUMBER_UUID:
    case SOFTWARE_REV_UUID:
      {
        uint16 len = strlen((char *)(pAttr->pValue));

        // verify offset
        if (offset > len)
        {
          status = ATT_ERR_INVALID_OFFSET;
        }
        else
        {
          // determine read length (exclude null terminating character)
          *pLen = MIN(maxLen, (len - offset));

          // copy data
          memcpy(pValue, &(pAttr->pValue[offset]), *pLen);
        }
      }
      break;

    case DEVICE_TYPE_UUID:
      // verify offset
      if (offset > sizeof(devInfoType))
      {
        status = ATT_ERR_INVALID_OFFSET;
      }
      else
      {
        // determine read length
        *pLen = MIN(maxLen, (sizeof(devInfoType) - offset));

        // copy data
        memcpy(pValue, &devInfoType[offset], *pLen);
      }
      break;

    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return ( status );
}


/*********************************************************************
*********************************************************************/
