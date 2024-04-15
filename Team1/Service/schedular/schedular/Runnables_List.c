/*
 * Runnables_List.c
 *
 *  Created on: Mar 14, 2024
 *      Author: Dell
 */
#include "Runnables_List.h"
#include "scheduler.h"
extern void LCD_TASK(void);
extern void APP_Control(void);
extern void SWITCH_Runnable(void);
extern void TimeDate_Update(void);
extern void StopWatch_TestAPP(void);
extern void APP_Control(void);
extern void APP_Send(void);

const runnable_t Runnablesinner[_Runnables_NUM]={
	[LCD_Runnable]={.name={"LCD"},.Delay_ms=0, .periodicity_ms=2, .cb=LCD_TASK} ,
	[SWITCH]={.name={"Switch Runnable"},.Delay_ms=100, .periodicity_ms=5, .cb=SWITCH_Runnable} ,
	[StopWatch]={.name={"StopWatch Runnable"},.Delay_ms=100, .periodicity_ms=100, .cb=StopWatch_TestAPP} ,
	[APP_send]={.name={"send pressed swiches Runnable"},.Delay_ms=200, .periodicity_ms=150, .cb=APP_Send} ,
	[APP_receive]={.name={"LCDTest"},.Delay_ms=100, .periodicity_ms=125, .cb=APP_Control},
	[TimeDateUpdate]={.name={"LCDTest"},.Delay_ms=100, .periodicity_ms=1000, .cb=TimeDate_Update}
};
