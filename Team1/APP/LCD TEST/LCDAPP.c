/*
 * LCDAPP.c
 *
 *  Created on: Apr 2, 2024
 *      Author: Dell
 */
#include "std_types.h"
#include "LCD.h"
#include "LED.h"

void stringcbf(void)
{
	LED_SetStatus(LED_RED, LED_STATE_ON);
}

void LCD_TestAPP(void)
{
	u32 static counter=0;
	    counter++;
	    if(counter==1)
	    {
	    	LCD_enuWriteString_asynch("Hodaaaaa",stringcbf);

	   }
	   else if(counter==2)
	    {
		   LCD_ClearScreen_async();
	    }

	    else if(counter==3)
	    {
	    	LCD_GotoPos_XY_async(1,0,NULLPTR);
	    }
	    else if(counter==4)
	    {
	    	LCD_enuWriteString_asynch("ITI",stringcbf);
	    }
}
