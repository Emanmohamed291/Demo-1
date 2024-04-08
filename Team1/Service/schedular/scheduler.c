/*
 * Scheduler.c
 *
 * Created: 3/14/2024 9:00:05 AM
 *  Author: Eman
 */ 
#include "Scheduler.h"
#include "Scheduler_Runnable.h"
#include "MCAL/03_SYSTICK/SYSTICK.h"

#define SCHEDULER_TIME           1

static volatile u32 pendingTicks = 0;
extern const runnable_t SYSRunnables[_Runnables_Num];

static void Sched(void);
static void Sched_TickCBF(void);

void Sched(void){
    static u32 timestamp = 0;
    u8 iter;
    for(iter=0; iter<_Runnables_Num; iter++){
        if( (SYSRunnables[iter].CBfunc) && (timestamp%SYSRunnables[iter].periodicitymS==0) ){
            SYSRunnables[iter].CBfunc();
        }
        timestamp += SCHEDULER_TIME;
    }
}

void Sched_TickCBF(void){
    pendingTicks++;
}
void Sched_Init(void){
    STK_Init(SOURCE_AHB_DIV8);
    STK_EnableSTKInterrupt();
    STK_SetTime_mS(SCHEDULER_TIME);
    STK_SetCallback(Sched_TickCBF);
    
}
void Sched_Start(void){
    STK_Start();
    while(1){
        if(pendingTicks){
            pendingTicks--;
            Sched();
        }
    }
}
