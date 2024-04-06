/*
 * Runnables_List.c
 *
 *  Created on: Mar 14, 2024
 *      Author: Dell
 */
#include "Runnables_List.h"
#include "scheduler.h"
extern void LCD_TASK(void);
extern void LCD_TestAPP(void);
const runnable_t RunnablesList[_Runnables_NUM]={
	   [LCD_Runnable]={.name={"LCD"}, .periodicity_ms=2, .cb=LCD_TASK}
	 , [LCD_TestRunnable]={.name={"LCDTest"}, .periodicity_ms=1000, .cb=LCD_TestAPP}
};
