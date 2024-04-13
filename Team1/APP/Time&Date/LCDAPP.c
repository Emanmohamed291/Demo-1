/*
 * LCDAPP.c
 *
 *  Created on: Apr 2, 2024
 *      Author: Dell
 */
#include "std_types.h"
#include "LCD.h"
#include "LED.h"
#include "SWITCH.h"
#include "TimeDate.h"

void testdisplay(void)
{
	LCD_DisableCursor_asynch();
	 TimeDateDisplay();
}
void testmovecursor(void)
{
	static u8 SW1_State=SWITCH_NOT_PRESSED;
	SWITCH_GetStatus(SWITCH_ONE, &SW1_State);
	static u8 SW2_State=SWITCH_NOT_PRESSED;

    if(SW1_State==SWITCH_PRESSED)
    {
    	LCD_EnableCursor_asynch();
    	LCD_MOVE_DIRECTION(Cursor_Right_Shift);
    	// Turn On Led
    	LED_SetStatus(LED_RED, LED_STATE_ON);
   }
    SWITCH_GetStatus(SWITCH_TWO, &SW2_State);
	if(SW2_State==SWITCH_PRESSED)
	{
		LCD_EnableCursor_asynch();
		LCD_INCDEC_TimeDate(Increment);
		// Turn OFF Led
	 	LED_SetStatus(LED_RED, LED_STATE_OFF);
	}
 }
