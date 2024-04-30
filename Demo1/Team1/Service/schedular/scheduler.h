/*
 * Scheduler.h
 *
 * Created: 3/14/2024 9:00:18 AM
 *  Author: Eman
 */ 


#ifndef SCHEDULER_H_
#define SCHEDULER_H_

/************************************************************************************
 *                                       datatypes                                  *
 * **********************************************************************************/
// typedef unsigned char         u8;
// typedef signed char           s8;
// typedef short unsigned int    u16;
// typedef short signed int      s16;
// typedef unsigned int          u32;
// typedef signed int            s32;
#include "STD_LIB/std_types.h"
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