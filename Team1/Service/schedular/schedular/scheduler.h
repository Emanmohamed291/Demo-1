/*
 * Scheduler.h
 *
 * Created: 3/14/2024 9:00:18 AM
 *  Author: Eman
 */ 


#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "STD_LIB/std_types.h"
/************************************************************************************
 *                                       datatypes                                  *
 * **********************************************************************************/

typedef void(*runnableCBF_t)(void);

typedef struct{
	char* name;
    u32 delay_ms;
	u32 periodicitymS;
	runnableCBF_t CBfunc;
}runnable_t;
/************************************************************************************
 *                                       functions                                  *
 * **********************************************************************************/

void Sched_Init(void);
void Sched_Start(void);
#endif /* SCHEDULER_H_ */