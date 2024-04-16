/*
 * APP_Demo.c
 *
 *  Created on: Apr 10, 2024
 *  Author: Eman
 */

/********************************************************************************************************/
/************************************************Includes************************************************/
/********************************************************************************************************/
#include "APP_Demo.h"

/********************************************************************************************************/
/************************************************Types***************************************************/
/********************************************************************************************************/
typedef enum
{
	ON,
	OFF
} EditState_t;
EditState_t EditState = OFF;

typedef enum
{
	MainMenu,
	DateAndTime,
	StopWatch
} LCD_Mode_t;

/********************************************************************************************************/
/************************************************Variables***********************************************/
/********************************************************************************************************/
static LCD_Mode_t LCD_Mode = MainMenu;
static LCD_Mode_t LCD_modechoose=StopWatch;;

u8 IPC_buffer;

u8 EditCount = 0;
u8 TimeDateDisp_flag = 0;
u8 CursorOn_flag = 0;
extern u8 X_Pos;
extern u8 Y_Pos;

extern StopWatch_value_t StopWatch_value;
static u8 StartStopCount = 0;
static u8 PauseContinueCount = 0;
static u8 StopWatchDisp_flag = 0;
static u8 StartStop_flag = 0;
static u8 PauseContinue_flag = 0;
static u8 StartFirst_flag = 1;

/********************************************************************************************************/
/*********************************************APIs Implementation****************************************/
/********************************************************************************************************/

/************************************************** Main Menu *******************************************/
void APP_LCDMainMenu(void)
{
	LCD_GotoPos_XY_async(0, 0, NULLPTR);
	LCD_enuWriteString_asynch(">>Stopwatch", NULLPTR);
	LCD_GotoPos_XY_async(1, 0, NULLPTR);
	LCD_enuWriteString_asynch(">>Date & Time", NULLPTR);
	LCD_GotoPos_XY_async(0, 0, NULLPTR);
}

/************************************************* cbf ***************************************************/
void CBFun(void)
{
	switch (LCD_Mode)
	{
	case MainMenu:
	{

		switch (IPC_buffer)
		{
		case UP:

			LCD_GotoPos_XY_async(0, 0, NULLPTR);
			LCD_modechoose = StopWatch;
			break;
		case DOWN_PAUSE_CONTINUE_STOPWATCH:

			LCD_GotoPos_XY_async(1, 0, NULLPTR);
			LCD_modechoose = DateAndTime;
			break;
		case OK_MODE:
			LCD_Mode = LCD_modechoose;
			LCD_ClearScreen_async();
			if (LCD_modechoose == DateAndTime)
			{
				// Date&Time Display
				TimeDateDisp_flag = 1;
				StopWatchDisp_flag = 0;
			}
			else if (LCD_modechoose == StopWatch)
			{
				// stopwatch display
				LCD_GotoPos_XY_async(0,3,NULLPTR);
	            LCD_enuWriteString_asynch("Stopwatch", NULLPTR);
				StopWatchDisp_flag = 1;
				TimeDateDisp_flag = 0;
			}
			break;
		default:
			break;
		}
	}
	break;
	case DateAndTime:
	{
		switch (IPC_buffer)
		{
		case OK_MODE:
			LCD_Mode = StopWatch;
			LCD_ClearScreen_async();
			TimeDateDisp_flag = 0;
			CursorOn_flag =0;
			// display StopWatch
			LCD_GotoPos_XY_async(0,3,NULLPTR);
	        LCD_enuWriteString_asynch("Stopwatch", NULLPTR);
			StopWatchDisp_flag = 1;
			break;
		case EDIT:
			if (EditCount % 2 == 0)
			{
				// Enable Cursor
				CursorOn_flag = 1;
				EditState = ON;
				EditCount++;
			}
			else
			{
				// Disable Cursor
				CursorOn_flag = 0;
				EditState = OFF;
				EditCount++;
			}
			break;
		case LEFT_RESET_STOPWATCH:
			if (EditState == ON)
			{
				LCD_MOVE_DIRECTION(Cursor_Left_Shift);
			}
			else
			{
			}
			break;
		case RIGHT_START_STOP_STOPWATCH:
			if (EditState == ON)
			{
			  LCD_MOVE_DIRECTION(Cursor_Right_Shift);
			}
			else
			{
			}
			break;
		case UP:
			if (EditState == ON)
			{
				LCD_INCDEC_TimeDate(Increment);
			}
			else
			{
			}
			break;
		case DOWN_PAUSE_CONTINUE_STOPWATCH:
			if (EditState == ON)
			{
				LCD_INCDEC_TimeDate(Decrement);
			}
			else
			{
			}
			break;
		default:
			
			break;
		}
	}
	break;

	case StopWatch:
	{
		switch (IPC_buffer)
		{

		case OK_MODE:
			LCD_Mode = DateAndTime;
			LCD_ClearScreen_async();
			// display time and date
			TimeDateDisp_flag = 1;
			StopWatchDisp_flag = 0;
			break;
		case RIGHT_START_STOP_STOPWATCH:
			PauseContinue_flag = 0;
			if (StartStopCount % 2 == 0)
			{
				StartStop_flag = 1;
				StartStopCount++;
			}
			else
			{
				StartStop_flag = 0;
				StartStopCount++;
			}
			break;
		case LEFT_RESET_STOPWATCH:
			StopWatch_Reset();
			StartStop_flag = 0;
			PauseContinue_flag = 0;
			break;
		case DOWN_PAUSE_CONTINUE_STOPWATCH:
			StartStop_flag = 0;
			if (PauseContinueCount % 2 == 0)
			{
				PauseContinue_flag = 0;
				PauseContinueCount++;
			}
			else
			{
				PauseContinue_flag = 1;
				PauseContinueCount++;
			}
			break;
		default:
		StopWatchDisp_flag = 1;
			break;
		}
	}
	break;
	default:
		break;
	}
}
/************************************************** Stop Watch *******************************************/
/* runs every 100 ms */
void StopWatch_TestAPP(void)
{
	if ((StartStop_flag == 1))
	{
		StopWatch_Count(&StopWatch_value);

		if (StartFirst_flag == 0)
		{
			StopWatch_Reset();
			StopWatch_Count(&StopWatch_value);
			StartFirst_flag = 1;
		}
	}

	else if (StartStop_flag == 0)
	{
		StartFirst_flag = 0;
	}

	if (PauseContinue_flag == 0)
	{
	}
	else if (PauseContinue_flag == 1)
	{
		StopWatch_Count(&StopWatch_value);
	}

	if (StopWatchDisp_flag == 1)
	{
		LCD_DisableCursor_asynch();
		StopWatch_Display();
	}
}

/*********************************************** app receive from switches ********************************/
void APP_Control(void)
{
	IPC_ReceiveUSART(USART_CH1, &IPC_buffer, 1, CBFun);
}

/*********************************************** app send pressed switches ********************************/
void APP_Send(void)
{
	u8 switchState = SWITCH_NOT_PRESSED;
	u8 buffer=0;
	/* OK_MODE */
	SWITCH_GetStatus(SWITCH_OK_MODE, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
	    buffer=OK_MODE;
		IPC_SendUSART(USART_CH1, &buffer, 1, NULLPTR);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
	/* UP */
	SWITCH_GetStatus(SWITCH_UP, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
        buffer=UP;
		IPC_SendUSART(USART_CH1, &buffer, 1, NULLPTR);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
	/* edit button */
	SWITCH_GetStatus(SWITCH_EDIT, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
	    buffer=EDIT;
		IPC_SendUSART(USART_CH1, &buffer, 1, NULLPTR);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
	/* RIGHT_START_STOP_STOPWATCH*/
	SWITCH_GetStatus(SWITCH_RIGHT_START_STOP_STOPWATCH, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
	    buffer=RIGHT_START_STOP_STOPWATCH;
		IPC_SendUSART(USART_CH1, &buffer, 1, NULLPTR);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
	/* LEFT_RESET_STOPWATCH */
	SWITCH_GetStatus(SWITCH_LEFT_RESET_STOPWATCH, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
	    buffer=LEFT_RESET_STOPWATCH;
		IPC_SendUSART(USART_CH1, &buffer, 1, NULLPTR);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
	/* DOWN_PAUSE_CONTINUE_STOPWATCH */
	SWITCH_GetStatus(SWITCH_DOWN_PAUSE_CONTINUE_STOPWATCH, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
	    buffer=DOWN_PAUSE_CONTINUE_STOPWATCH;
		IPC_SendUSART(USART_CH1, &buffer, 1, NULLPTR);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
}
/*********************************************** to update time values ********************************/
/* runs every 1000 ms */
void TimeDate_Update(void)
{
	TimeDate_Running();
	if (TimeDateDisp_flag == 1)
	{
		TimeDateDisplay();
	}
	if (CursorOn_flag == 1)
	{
		LCD_EnableCursor_asynch();
		LCD_GotoPos_XY_async(X_Pos, Y_Pos, NULLPTR);
	}
}
