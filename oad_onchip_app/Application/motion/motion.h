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
#include <ti/drivers/PIN.h>
#include <device_common.h>

/*********************************************************************
 * TYPEDEFS
 */

typedef enum
{
    MOTION_INIT,
    MOTION_CALIBRATE,
    MOTION_DISABLE,
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
extern void CustomDevice_init(uint8_t selfEntity);
extern void CustomDevice_processApplicationMessage(sbpEvt_t *pMsg);

#endif
#endif /* APPLICATION_MOTION_H_ */
