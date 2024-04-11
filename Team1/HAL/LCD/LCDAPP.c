/*
 * LCDAPP.c
 *
 *  Created on: Apr 2, 2024
 *      Author: Dell
 */
#include "STD_LIB/std_types.h"
#include "LCD.h"
#include "HAL/LED/LED.h"
#include "APP/StopWatch/StopWatch.h"
#include "APP/Date/Date.h"
typedef enum{
	continue_state,
	pause_state,
	reset_state
}state_t;
state_t state;

extern Date_value_t Date_value;
void Date_TestAPP(void)
{
    LCD_GotoPos_XY_async(1,0,NULLPTR);
    LCD_enuWriteNumber_asynch(Date_value.Day_index2, NULLPTR);
    LCD_enuWriteNumber_asynch(Date_value.Day_index1, NULLPTR);
    LCD_enuWriteString_asynch("/", NULLPTR);
    LCD_enuWriteNumber_asynch(Date_value.Month_index2, NULLPTR);
    LCD_enuWriteNumber_asynch(Date_value.Month_index1, NULLPTR);
    LCD_enuWriteString_asynch("/", NULLPTR);
    LCD_enuWriteNumber_asynch(Date_value.Year_index4, NULLPTR);
    LCD_enuWriteNumber_asynch(Date_value.Year_index3, NULLPTR);
    LCD_enuWriteNumber_asynch(Date_value.Year_index2, NULLPTR);
    LCD_enuWriteNumber_asynch(Date_value.Year_index1, NULLPTR);
}

void stringcbf(void)
{
	LED_SetStatus(LED_RED, LED_STATE_ON);
}
StopWatch_value_t StopWatch_value;
void StopWatch_TestAPP(void)
{
	static u8 count = 0;
	static u8 change = 1;
	count++;
	// if(count == 200)
	// 	state = reset_state;
	change = 1;
	switch(state){
		case continue_state:
	    StopWatch_Count(&StopWatch_value);
		if(StopWatch_value.hour == 0){
			LCD_GotoPos_XY_async(0,0,NULLPTR);
			LCD_enuWriteString_asynch("0", NULLPTR);
			LCD_enuWriteString_asynch("0", NULLPTR);
			LCD_enuWriteString_asynch(":", NULLPTR);
		}
		else if(StopWatch_value.hour < 10){
			LCD_GotoPos_XY_async(0,0,NULLPTR);
			LCD_enuWriteString_asynch("0", NULLPTR);
			LCD_enuWriteNumber_asynch(StopWatch_value.hour, NULLPTR);
			LCD_enuWriteString_asynch(":", NULLPTR);
		}
		else{
			LCD_GotoPos_XY_async(0,0,NULLPTR);
			LCD_enuWriteNumber_asynch(StopWatch_value.hour, NULLPTR);
			LCD_enuWriteString_asynch(":", NULLPTR);
		}
		if(StopWatch_value.min == 0){
			LCD_GotoPos_XY_async(0,3,NULLPTR);
			LCD_enuWriteString_asynch("0", NULLPTR);
			LCD_enuWriteString_asynch("0", NULLPTR);
			LCD_enuWriteString_asynch(":", NULLPTR);
		}
		else if(StopWatch_value.min < 10){
			LCD_GotoPos_XY_async(0,3,NULLPTR);
			LCD_enuWriteString_asynch("0", NULLPTR);
			LCD_enuWriteNumber_asynch(StopWatch_value.min, NULLPTR);
			LCD_enuWriteString_asynch(":", NULLPTR);
		}
		else{
			LCD_GotoPos_XY_async(0,3,NULLPTR);
			LCD_enuWriteNumber_asynch(StopWatch_value.min, NULLPTR);
			LCD_enuWriteString_asynch(":", NULLPTR);
		}
		if(StopWatch_value.sec == 0){
			LCD_GotoPos_XY_async(0,6,NULLPTR);
			LCD_enuWriteString_asynch("0", NULLPTR);
			LCD_enuWriteString_asynch("0", NULLPTR);
			LCD_enuWriteString_asynch(":", NULLPTR);
		}
		else if(StopWatch_value.sec < 10){
			LCD_GotoPos_XY_async(0,6,NULLPTR);
			LCD_enuWriteString_asynch("0", NULLPTR);
			LCD_enuWriteNumber_asynch(StopWatch_value.sec, NULLPTR);
			LCD_enuWriteString_asynch(":", NULLPTR);
		}
		else{
			LCD_GotoPos_XY_async(0,6,NULLPTR);
			LCD_enuWriteNumber_asynch(StopWatch_value.sec, NULLPTR);
			LCD_enuWriteString_asynch(":", NULLPTR);
		}
		if(StopWatch_value.millisec == 0){
			LCD_GotoPos_XY_async(0,9,NULLPTR);
			LCD_enuWriteString_asynch("0", NULLPTR);
		}
		else{
			LCD_GotoPos_XY_async(0,9,NULLPTR);
			LCD_enuWriteNumber_asynch(StopWatch_value.millisec, NULLPTR);
		}
	break;
	case pause_state:
	    change = 1;
		break;
	case reset_state:
	if(change == 1){
		change = 0;
		StopWatch_Reset();
		LCD_GotoPos_XY_async(0,0,NULLPTR);
		LCD_enuWriteString_asynch("0", NULLPTR);
		LCD_enuWriteString_asynch("0", NULLPTR);
		LCD_enuWriteString_asynch(":", NULLPTR);
		LCD_GotoPos_XY_async(0,3,NULLPTR);
		LCD_enuWriteString_asynch("0", NULLPTR);
		LCD_enuWriteString_asynch("0", NULLPTR);
		LCD_enuWriteString_asynch(":", NULLPTR);
		LCD_GotoPos_XY_async(0,6,NULLPTR);
		LCD_enuWriteString_asynch("0", NULLPTR);
		LCD_enuWriteString_asynch("0", NULLPTR);
		LCD_enuWriteString_asynch(":", NULLPTR);
		LCD_GotoPos_XY_async(0,9,NULLPTR);
		LCD_enuWriteString_asynch("0", NULLPTR);
	}
	else{

	}
		break;
	default:
		break;
	}
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
			LCD_enuWriteNumber_asynch(123, stringcbf);
	    }
}
