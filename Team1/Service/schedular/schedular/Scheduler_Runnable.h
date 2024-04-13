/*
 * Scheduler_Runnable.h
 *
 * Created: 3/14/2024 9:00:45 AM
 *  Author: Eman
 */ 


#ifndef SCHEDULER_RUNNABLE_H_
#define SCHEDULER_RUNNABLE_H_


typedef enum{
    LCD_Runnable,
	LCD_TestRunnable,
    date_TestRunnable,
    SWITCH,
    DEMO_APP_Control,
    APP_sendtask,
    APP_Demo1,
    _Runnables_Num
}Runnable_t;


#endif /* SCHEDULER_RUNNABLE_H_ */