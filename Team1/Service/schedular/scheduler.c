/*
 * scheduler.c
 *
 *  Created on: Mar 12, 2024
 *      Author: Dell
 */

#include "scheduler.h"
#include "Runnables_List.h"
#include "STK.h"


#define SCHED_TICK_TIME_ms 1
static void scheduler_tickcb(void);
static void scheduler(void);

// variable to determine if we should call scheduler
static volatile u32 pendingTicks=0;
extern const runnable_t RunnablesList[_Runnables_NUM];
/*For any internal configuration
typedef struct
{
	runnable_t *RUNNABLE;
	u32 RemainTime_ms;
}runnableinfo_t;
runnableinfo_t rinfo[MAX_SUPPORTED_TASKS];*/

static void scheduler_tickcb(void)
{
	pendingTicks++;
}

static void scheduler(void)
{
	u32 idx=0;
	static u32 timestamp=0;
	for(idx = 0; idx < _Runnables_NUM; idx++)
	{
		if(RunnablesList[idx].cb && (timestamp%RunnablesList[idx].periodicity_ms==0))
		{
			RunnablesList[idx].cb();
		}
	}
	timestamp+=SCHED_TICK_TIME_ms;
}
void scheduler_init(void)
{
	// init vars (if needed)
	STK_Init();
	// systick configure
	STK_SetTime_ms(SCHED_TICK_TIME_ms);
	STK_RegisterCallback(scheduler_tickcb);

}

void scheduler_start(void)
{
	// start systick timer
	STK_Start(STK_Periodic);
	while(1)
	{

		if(pendingTicks)
		{
			pendingTicks--;
			scheduler();
		}
		else
		{

		}
	}
}

/*Sched_ErrorStatus_t scheduler_registerrunnable(runnable_t* runnable)
{
	Sched_ErrorStatus_t Ret_Errorstate=Sched_OK;
	if(runnable && (rinfo[runnable->priority].RUNNABLE == NULLPTR))
	{
		rinfo[runnable->priority].RUNNABLE = runnable;
		rinfo[runnable->priority].RemainTime_ms = runnable->Delay_ms;
	}
	else
	{
		Ret_Errorstate=Sched_NOK;
	}
	return Ret_Errorstate;
}*/

