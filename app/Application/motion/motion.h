/*
 * motion.h
 *
 *  Created on: 17 квіт. 2020 р.
 *      Author: Oleh
 */

#ifndef APPLICATION_MOTION_H_
#define APPLICATION_MOTION_H_
#if MOTION_COMPILE



/*********************************************************************
 * INCLUDES
 */
#include <device_common.h>
#include <ti/drivers/PIN.h>

/*********************************************************************
 * TYPEDEFS
 */

typedef enum
{
    MOTION_INIT,
    MOTION_WAIT_CONNECT,
    MOTION_DISABLED,
    MOTION_CALIBRATE,
    MOTION_MEASURE,
    MOTION_DETECT,
    MOTION_STATE_MAX
} motionState_t;

/*********************************************************************
 * MACROS
 */
#define ADVERT_DATA_NAME                'M', 'o', 't', 'i', 'o', 'n'
#define ADVERT_DATA_NAME_DISPLAY_LEN    (7)  // and terminal zero '\0'

/*********************************************************************
 *  EXTERNAL VARIABLES
 */

/*********************************************************************
 * API FUNCTIONS
 */
extern void CustomDevice_hardwareInit(void);
extern void CustomDevice_bleInit(uint8_t selfEntity);
extern void CustomDevice_processApplicationMessage(Msg_t *pMsg);



#endif
#endif /* APPLICATION_MOTION_H_ */
