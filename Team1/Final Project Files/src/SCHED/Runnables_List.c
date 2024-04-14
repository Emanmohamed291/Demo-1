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

const runnable_t RunnablesList[_Runnables_NUM]={
	[LCD_Runnable]={.name={"LCD"}, .periodicity_ms=2, .cb=LCD_TASK} ,
	[SWITCH]={.name={"Switch Runnable"}, .periodicity_ms=5, .cb=SWITCH_Runnable} ,
	[StopWatch]={.name={"StopWatch Runnable"}, .periodicity_ms=100, .cb=StopWatch_TestAPP} ,
	[APP_send]={.name={"send pressed swiches Runnable"}, .periodicity_ms=150, .cb=APP_Send} ,
	[APP_receive]={.name={"LCDTest"}, .periodicity_ms=125, .cb=APP_Control},
	[TimeDateUpdate]={.name={"LCDTest"}, .periodicity_ms=1000, .cb=TimeDate_Update}
};
