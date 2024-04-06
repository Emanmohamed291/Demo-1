/*
 * scheduler.h
 *
 *  Created on: Mar 12, 2024
 *      Author: Dell
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_
#include "std_types.h"

typedef void (*runnable_cb_t) (void);

typedef enum
{
	Sched_OK,
	Sched_NOK
}Sched_ErrorStatus_t;

// for any configurations configurable by the user
typedef struct
{
	char *name;
	//u32 Delay_ms;
	u32 periodicity_ms;
	//u32 priority;
	runnable_cb_t cb;
} runnable_t;

//Sched_ErrorStatus_t scheduler_registerrunnable(runnable_t *runnable);

// To enable timers
void scheduler_init(void);
// all tasks must be created between the init and the start
// Although our implementation will allow tasks to be run after starting sched
void scheduler_start(void);

#endif /* SCHEDULER_H_ */
