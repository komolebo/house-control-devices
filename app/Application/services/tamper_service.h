/*
 * tamper_service.h
 *
 *  Created on: 2 трав. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_SERVICES_TAMPER_SERVICE_H_
#define APPLICATION_SERVICES_TAMPER_SERVICE_H_



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
typedef void (*TamperServiceChange_t)(uint16_t connHandle, uint8_t paramID,
                                      uint16_t len, uint8_t *pValue);

typedef struct
{
    TamperServiceChange_t pfnChangeCb;          // Called when characteristic value changes
    TamperServiceChange_t pfnCfgChangeCb;       // Called when characteristic CCCD changes
} TamperServiceCBs_t;

/*********************************************************************
 * API FUNCTIONS
 */

/*
 * TamperService_AddService- Initializes the TamperService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t TamperService_AddService(uint8_t rspTaskId);

/*
 * TamperService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t TamperService_RegisterAppCBs(TamperServiceCBs_t *appCallbacks);

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
extern bStatus_t TamperService_SetParameter(uint8_t param,
                                            uint16_t len,
                                            void *value);

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
extern bStatus_t TamperService_GetParameter(uint8_t param,
                                            uint16_t *len,
                                            void *value);

#endif /* APPLICATION_SERVICES_TAMPER_SERVICE_H_ */
