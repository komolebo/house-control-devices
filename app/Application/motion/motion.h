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
// Struct for message about button state
typedef struct
{
    PIN_Id pinId;
    uint8_t state;
} pzButtonState_t;

/*********************************************************************
 * MACROS
 */
#define ADVERT_DATA_NAME                'M', 'o', 't', 'i', 'o', 'n'
#define ADVERT_DATA_NAME_DISPLAY_LEN    (7)  // and terminal zero '\0'

/*********************************************************************
 *  EXTERNAL VARIABLES
 */
// Advertisement data
extern uint8_t advertData[ADVERT_DATA_NAME_DISPLAY_LEN + ADVERT_DATA_HEADER_LEN];


/*********************************************************************
 * API FUNCTIONS
 */
extern void CustomDevice_hardwareInit(void);
extern void CustomDevice_bleInit(uint8_t selfEntity);
extern void CustomDevice_processApplicationMessage(Msg_t *pMsg);



#endif
#endif /* APPLICATION_MOTION_H_ */
