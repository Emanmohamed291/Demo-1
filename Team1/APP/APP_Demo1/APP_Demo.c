/*
 * APP_Demo.c
 *
 *  Created on: Apr 10, 2024
 *  Author: Eman
 */
#include "STD_LIB/std_types.h"
#include "LCD.h"
#include "HAL/LED/LED.h"
#include "MCAL/04_USART/USART.h"
#include "APP/StopWatch/StopWatch.h"
#include "APP/StopWatch/StopWatch.h"
#include "APP/Date/Date.h"
#include "HAL/SubPub/PubSub.h"
#include "HAL/SWITCH/SWITCH.h"

/* main menu */
#define OK                          '1'
#define MODE                        '1'

#define UP                          '2'
#define EDIT                        '3'

#define RIGHT                       '4'
#define START_STOP_STOPWATCH        '4'

#define LEFT                        '5'
#define RESET_STOPWATCH             '5'

#define PAUSE_CONTINUE_STOPWATCH    '6'
#define DOWN                        '6'



typedef enum{
	continue_state,
	pause_state,
	reset_state,
	start_state,
	stop_state
}state_t;
state_t state;

u8 PubSub_buffer;
extern Date_value_t Date_value;
// void Date_TestAPP(void)
// {
//     LCD_GotoPos_XY_async(1,0,NULLPTR);
//     LCD_enuWriteNumber_asynch(Date_value.Day_index2, NULLPTR);
//     LCD_enuWriteNumber_asynch(Date_value.Day_index1, NULLPTR);
//     LCD_enuWriteString_asynch("/", NULLPTR);
//     LCD_enuWriteNumber_asynch(Date_value.Month_index2, NULLPTR);
//     LCD_enuWriteNumber_asynch(Date_value.Month_index1, NULLPTR);
//     LCD_enuWriteString_asynch("/", NULLPTR);
//     LCD_enuWriteNumber_asynch(Date_value.Year_index4, NULLPTR);
//     LCD_enuWriteNumber_asynch(Date_value.Year_index3, NULLPTR);
//     LCD_enuWriteNumber_asynch(Date_value.Year_index2, NULLPTR);
//     LCD_enuWriteNumber_asynch(Date_value.Year_index1, NULLPTR);
// }
/************************************************** Init *******************************************/
// void initcbf(void)
// {
// 	//LED_SetStatus(LED_GREEN, LED_STATE_ON);
// }
// void APP_Init(void){
// 	RCC_enuEnableDisablePeripheral(RCC_AHB1,GPIOAEN,Periph_enuON);
// 	RCC_enuEnableDisablePeripheral(RCC_AHB1,GPIOBEN,Periph_enuON);
// 	LCD_InitPins();
// 	LCD_init_asynch(initcbf);
// 	SWITCH_Init();
// 	PubSub_Init(USART_CH1);
// }
/************************************************** Main Menu *******************************************/
void APP_LCDMainMenu(void)
{
    LCD_GotoPos_XY_async(0,0,NULLPTR);
    LCD_enuWriteString_asynch("->>Stop Watch",NULLPTR);
    LCD_GotoPos_XY_async(1,0,NULLPTR);
    LCD_enuWriteString_asynch("->>Date & Time",NULLPTR);
    LCD_GotoPos_XY_async(0,0,NULLPTR);
}

typedef enum{
    MainMenu,
    DateAndTime,
    StopWatch
}LCD_Mode_t;

u8 LCD_CursorPos = 0; 
static LCD_Mode_t LCD_Mode;
static LCD_Mode_t LCD_modechoose;
/************************************************* cbf ***************************************************/
void CBFun(void)
{
    switch (LCD_Mode)
    {
    case MainMenu:
        {
            /* code */
            switch (PubSub_buffer)
            {
            case UP:
                /* code */
                LCD_GotoPos_XY_async(0,0,NULLPTR);
                LCD_modechoose = StopWatch;
            break;
            case DOWN:
                /* code */
                LCD_GotoPos_XY_async(1,0,NULLPTR);
                LCD_modechoose = DateAndTime;
            break;
			case OK:
                /* code */
                LCD_Mode = LCD_modechoose;
            break;
            default:
            break;
            }

        }
    break;
    case DateAndTime:
        /* code */
        break;
    case StopWatch:{
		switch (PubSub_buffer)
		{
		case MODE:
		    LCD_Mode = DateAndTime;
			break;
		default:
			state = PubSub_buffer;
			StopWatch_TestAPP();
			break;
		}
	}
        break;
    default:
        break;
    }
}
/************************************************** Stop Watch *******************************************/
extern StopWatch_value_t StopWatch_value;
void StopWatch_TestAPP(void)
{
	static u8 count = 0;
	static u8 change = 1;
	count++;
	switch(state){
		case continue_state:
		change = 1;
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
	case start_state:
	    change = 1;
		StopWatch_Reset();
		state = continue_state; 
		break;
	case stop_state:
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

/*********************************************** app receive from switches ********************************/
void APP_Control(void)
{
    PubSub_ReceiveUSART(USART_CH1, &PubSub_buffer, 1, CBFun);
}

/*********************************************** app receive from switches ********************************/
void APP_Send(void)
{
	PubSub_SendUSART(USART_CH1, MODE, 1, CBFun);
	u8 switchState = SWITCH_NOT_PRESSED;
	/* mode */
	SWITCH_GetStatus(SWITCH_MODE, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
		PubSub_SendUSART(USART_CH1, MODE, 1, CBFun);
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
		PubSub_SendUSART(USART_CH1, UP, 1, CBFun);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
	/* ok button */
	SWITCH_GetStatus(SWITCH_Ok, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
		PubSub_SendUSART(USART_CH1, OK, 1, CBFun);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
	/* edit*/
	SWITCH_GetStatus(SWITCH_EDIT, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
		PubSub_SendUSART(USART_CH1, EDIT, 1, CBFun);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
	/* SWITCH_STARTSTOP_STOPWATCH */
	SWITCH_GetStatus(SWITCH_STARTSTOP_STOPWATCH, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
		PubSub_SendUSART(USART_CH1, START_STOP_STOPWATCH, 1, CBFun);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
	/* SWITCH_LEFT_RESET_STOPWATCH */
	SWITCH_GetStatus(SWITCH_LEFT_RESET_STOPWATCH, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
		PubSub_SendUSART(USART_CH1, LEFT, 1, CBFun);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
	/* SWITCH_DOWN_PAUSECONT_STOPWATCH */
	SWITCH_GetStatus(SWITCH_DOWN_PAUSECONT_STOPWATCH, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
		PubSub_SendUSART(USART_CH1, DOWN, 1, CBFun);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
	/* SWITCH_RIGHT */
	SWITCH_GetStatus(SWITCH_RIGHT, &switchState);
	switch (switchState)
	{
	case SWITCH_PRESSED:
		PubSub_SendUSART(USART_CH1, RIGHT, 1, CBFun);
		switchState = SWITCH_NOT_PRESSED;
		break;
	case SWITCH_NOT_PRESSED:
		break;
	default:
		break;
	}
}