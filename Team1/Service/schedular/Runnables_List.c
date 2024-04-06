/*
 * Runnables_List.c
 *
 *  Created on: Mar 14, 2024
 *      Author: Dell
 */
#include "Runnables_List.h"
#include "scheduler.h"
extern void SWITCH_Runnable(void);
extern void SWITCH_LED_Runnable(void);
//extern void ToggleRedRunnable(void);
//extern void TrafficLights_Runnable(void);
const runnable_t RunnablesList[_Runnables_NUM]={
	[SWITCH]={.name={"Switch Runnable"}, .periodicity_ms=5, .cb=SWITCH_Runnable},
	[switch_led]={.name={"SwitchLed Runnable"}, .periodicity_ms=150, .cb=SWITCH_LED_Runnable}
	//[ToggleRed]={.name={"ToggleRunnable"}, .periodicity_ms=1000, .cb=ToggleRedRunnable},
    //[Traffic_Light]={.name={"TrafficLight"}, .periodicity_ms=2000, .cb=TrafficLights_Runnable}
};
