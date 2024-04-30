/*
 * app2.c
 *
 *  Created on: Mar 20, 2024
 *      Author: Dell
 */
#include "std_types.h"
#include "RCC.h"
#include "LED.h"
#include "SWITCH.h"


void SWITCH_LED_Runnable(void)
{
	u8 SW_State;
	SWITCH_GetStatus(SWITCH_ONE, &SW_State);
    if(SW_State==SWITCH_PRESSED)
    {
    	// Turn On Led
    	LED_SetStatus(LED_RED, LED_STATE_ON);
    }
    else
    {
    	// Turn Off Led
    	LED_SetStatus(LED_RED, LED_STATE_OFF);

    }
}
