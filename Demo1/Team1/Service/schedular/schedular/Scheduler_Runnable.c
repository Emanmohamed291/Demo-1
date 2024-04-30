/*
 * Scheduler_Runnable.c
 *
 * Created: 3/14/2024 9:00:58 AM
 *  Author: Eman
 */ 
#include "Scheduler_Runnable.h"
#include "Scheduler.h"

extern void SWITCH_Runnable(void);
//extern void APP1_Runnable(void);
// extern void APP2_Runnable(void);
// extern void Traffic_Runnable(void);
extern void APP_Send(void);
extern void APP_Control(void);
extern void LCD_TASK(void);
extern void LCD_TestAPP(void);
extern void StopWatch_TestAPP(void);

const runnable_t SYSRunnables[_Runnables_Num] =
{
    [LCD_Runnable]={
        .name = "LCD",
        .delay_ms = 0,
	    .periodicitymS = 2,
	    .CBfunc = &LCD_TASK
    },
    [LCD_TestRunnable]={
        .name = "LCD2",
        .delay_ms = 0,
	    .periodicitymS = 100,
	    .CBfunc = &LCD_TestAPP
    },
    // [TRAFFICAPP]={
    //     .name = "traffic1",
    //     .delay_ms = 0,
	//     .periodicitymS = 1000,
	//     .CBfunc = &LCD_TestAPP
    // },
    // [APP1]={
    //     .name = "led1",
    //     .delay_ms = 20,
	//     .periodicitymS = 800,
	//     .CBfunc = &APP1_Runnable
    // },
    // [APP2]={
    //     .name = "led2 with switch",
    //     .delay_ms = 20,
	//     .periodicitymS = 600,
	//     .CBfunc = &APP2_Runnable
    // },
    // [APP_sendtask]={
    //     .name = "APP_sendtask",
    //     .delay_ms = 10,
	//     .periodicitymS = 100,
	//     .CBfunc = APP_Send
    // },
    [DEMO_APP_Control]={
        .name = "APP_control1",
        .delay_ms = 10,
	    .periodicitymS = 150,
	    .CBfunc = APP_Control
    },
    [SWITCH]={
        .name = "switch1",
        .delay_ms = 10,
	    .periodicitymS = 20,
	    .CBfunc = &SWITCH_Runnable
    }
};