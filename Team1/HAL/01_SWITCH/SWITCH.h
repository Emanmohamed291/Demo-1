/*
 * SWITCH.h
 *
 * Created: 2/18/2024 11:25:43 AM
 *  Author: Dell
 */ 


#ifndef SWITCH_H_
#define SWITCH_H_

#include "std_types.h"
#include "SWITCH_cfg.h"

#define SWITCH_INPUT_PU 1
#define SWITCH_INPUT_PD 0

#define SWITCH_PRESSED 1
#define SWITCH_NOT_PRESSED 0


/*** Switch configurations ***/
typedef struct{
	void* SWITCH_Port;
	u32 SWITCH_Pin;
	u32 SWITCH_Mode;
	}SWITCH_cfg_t;
	
/*** Error Status ***/
typedef enum
{
	SWITCH_OK,
	SWITCH_InvalidSwitch,
	SWITCH_NULLPTR
	}SWITCH_ERRORSTATUS_t;

/*** Function Prototypes ***/

/**
 * @brief: Initializes the SWITCH according to external SWITCH configuration.
 *
 * @return: SWITCH_ERRORSTATUS_t :
 *         - SWITCH_OK
 *         - GPIO_ERRORStatus
 */
SWITCH_ERRORSTATUS_t SWITCH_Init(void);

/**
 * @brief: Sets the status of the specified SWITCH.
 *
 * @param: - Copy_SWITCH: The name of the SWITCH whose status will be set.
 *         - Copy_Status: The status of the SWITCH (SWITCH_PRESSED, SWITCH_NOT_PRESSED).
 *
 * @return: SWITCH_ERRORSTATUS_t:
 *         - SWITCH_OK
 *         - SWITCH_InvalidSwitch
 *         - SWITCH_NULLPTR
 *         - GPIO_ERRORStatus
 */
SWITCH_ERRORSTATUS_t SWITCH_GetStatus(u32 Copy_SWITCH, u8* Copy_Status);
	





#endif /* SWITCH_H_ */
