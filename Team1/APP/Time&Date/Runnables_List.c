/*
 * Runnables_List.c
 *
 *  Created on: Mar 14, 2024
 *      Author: Dell
 */
#include "Runnables_List.h"
#include "scheduler.h"
extern void LCD_TASK(void);
extern void testdisplay(void);
extern void SWITCH_Runnable(void);
extern void testmovecursor(void);

const runnable_t RunnablesList[_Runnables_NUM]={
	   [LCD_Runnable]={.name={"LCD"}, .periodicity_ms=2, .cb=LCD_TASK} ,
	    [SWITCH]={.name={"Switch Runnable"}, .periodicity_ms=5, .cb=SWITCH_Runnable} ,
	    [LCD_TestCursor]={.name={"LCDTest"}, .periodicity_ms=150, .cb=testmovecursor},
	  [LCD_TestRunnable]={.name={"LCDTest"}, .periodicity_ms=1000, .cb=testdisplay}
};
