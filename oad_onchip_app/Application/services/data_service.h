/*
 * data_service.h
 *
 *  Created on: 18 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_MOTION_MOTION_DATA_SERVICE_H_
#define APPLICATION_MOTION_MOTION_DATA_SERVICE_H_

#if MOTION_COMPILE


/*********************************************************************
 * INCLUDES
 */
#include <profiles_if.h>

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*DataServiceChange_t)(uint16_t connHandle, uint8_t paramID,
                                    uint16_t len, uint8_t *pValue);

typedef struct
{
    DataServiceChange_t pfnChangeCb;          // Called when characteristic value changes
    DataServiceChange_t pfnCfgChangeCb;       // Called when characteristic CCCD changes
} DataServiceCBs_t;

/*********************************************************************
 * API FUNCTIONS
 */

/*
 * DataService_AddService- Initializes the DataService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t DataService_AddService(uint8_t rspTaskId);

/*
 * DataService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t DataService_RegisterAppCBs(DataServiceCBs_t *appCallbacks);

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
extern bStatus_t DataService_SetParameter(uint8_t param,
                                          uint16_t len,
                                          void *value);

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
extern bStatus_t DataService_GetParameter(uint8_t param,
                                          uint16_t *len,
                                          void *value);


#endif // MOTION_COMPILE
#endif /* APPLICATION_MOTION_MOTION_DATA_SERVICE_H_ */
