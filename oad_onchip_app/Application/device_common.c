/*
 * device_common.c
 *
 *  Created on: 13 трав. 2020 р.
 *      Author: Oleh
 */
#include "simple_peripheral_oad_onchip.h"
#include "device_common.h"

#include "peripheral.h"
#include "profiles_if.h"

/*********************************************************************
 * @fn     enqueueMsg
 *
 * @brief  Utility function that sends the event and data to the application.
 *         Handled in the task loop.
 *
 * @param  event    Event type
 * @param  pData    Pointer to message data
 */
bStatus_t enqueueMsg(uint8_t event, void *pData)
{
    return SimplePeripheral_enqueueMsg(event, 0, (uint8_t *)pData);
    /*uint8_t success;
    Msg_t *pMsg = ICall_malloc(sizeof(Msg_t));

    if(pMsg)
    {
        pMsg->event = event;
        pMsg->pData = pData;


        return (success) ? SUCCESS : FAILURE;
    }

    return(bleMemAllocError);*/
}

/*********************************************************************
 * @fn      isConnected
 *
 * @brief
 *
 * @param
 *
 * @return  TRUE if connected to central.
 *          TRUE if disconnected from central.
 */
bool isConnected(void)
{
    gaprole_States_t gapState;

    GAPRole_GetParameter(GAPROLE_STATE, &gapState);

    return gapState == GAPROLE_CONNECTED;
}
