/*
 * LED.h
 *
 * Created: 2/18/2024 11:25:43 AM
 *  Author: Dell
 */ 


#ifndef LED_H_
#define LED_H_

#include "std_types.h"
#include "LED_cfg.h"

#define LED_STATE_ON  0x00000001
#define LED_STATE_OFF 0X00010000
#define LED_CONN_FWD  0x00000000
#define LED_CONN_REV  0x00010001

/*** Led configurations ***/
typedef struct{
	void* LED_Port;
	u32 LED_Pin;
	u8  LED_Conn;
	u32 LED_Mode;
	u32 LED_Speed;
	}LED_cfg_t;
	
/*** Error Status ***/
typedef enum
{
	LED_OK,
	LED_InvalidLed,
	LED_InvalidStatus
	}LED_ERRORSTATUS_t;

/*** Function Prototypes ***/

/**
 * @brief: Initializes the LED according to external LED configuration.
 *
 * @return: LED_ERRORSTATUS_t :
 *         - LED_OK
 *         - GPIO_ERRORStatus
 */
LED_ERRORSTATUS_t LED_Init(void);

/**
 * @brief: Sets the status of the specified LED.
 *
 * @param: - Copy_LED: The name of the LED whose status will be set.
 *         - Copy_Status: The status to set for the LED (LED_STATE_ON or LED_STATE_OFF).
 *
 * @return: LED_ERRORSTATUS_t:
 *         - LED_OK
 *         - LED_InvalidLed
 *         - LED_InvalidStatus
 */
LED_ERRORSTATUS_t LED_SetStatus(u32 Copy_LED, u32 Copy_Status);
	




#endif /* LED_H_ */
