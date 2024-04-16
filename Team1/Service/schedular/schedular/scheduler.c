/*
 * NVIC.h
 *
 * Created: 3/3/2024 3:26:27 PM
 *  Author: Eman
 */ 

#include "scheduler.h"
#include "Runnables_List.h"
#include "MCAL/SYSTICK/STK.h"
#include "HAL/LED/LED.h"

typedef struct 
{
    runnable_t runnable  ;
    u32  remainingTime;
}runablevalue_t;

#define SCHED_TICK_TIME_ms 1U
static void scheduler_tickcb(void);
static void scheduler(void);

// variable to determine if we should call scheduler
static volatile u32 pendingTicks=0;
extern const runnable_t Runnablesinner[_Runnables_NUM];
runablevalue_t RunnablesList[_Runnables_NUM];
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
    u32 iter = 0;
    for(iter = 0; iter < _Runnables_NUM; iter++)
    {
        if(RunnablesList[iter].runnable.cb)
        {
            if(RunnablesList[iter].remainingTime == 0)
            {
				LED_SetStatus(LED_RED, LED_STATE_ON);
                RunnablesList[iter].runnable.cb();
                RunnablesList[iter].remainingTime = RunnablesList[iter].runnable.periodicity_ms;
            }
        }
        (RunnablesList[iter].remainingTime) -= SCHED_TICK_TIME_ms;
    }
}


void scheduler_init(void)
{
	u32 iter=0;
	// init vars (if needed)
	STK_Init();
	LED_Init();
	// systick configure
	STK_SetTime_ms(SCHED_TICK_TIME_ms);
	STK_RegisterCallback(scheduler_tickcb);
	for(iter = 0; iter < _Runnables_NUM; iter++)
	{
		 if(Runnablesinner[iter].cb)
		 {
			RunnablesList[iter].runnable.name = Runnablesinner[iter].name;
			RunnablesList[iter].runnable.periodicity_ms = Runnablesinner[iter].periodicity_ms;
		 	RunnablesList[iter].runnable.cb = Runnablesinner[iter].cb;
			RunnablesList[iter].remainingTime = Runnablesinner[iter].Delay_ms;
		}
	}

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
        // Optionally add a delay or yield to reduce CPU usage
        // You can use a delay function or sleep/yield function here
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

