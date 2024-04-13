/*
 * SWITCH_cfg.c
 *
 * Created: 2/18/2024 11:26:36 AM
 *  Author: Dell
 */ 

#include "GPIO.h"
#include "SWITCH.h"

/*** SWITCHS user configuration ***/
const SWITCH_cfg_t SWITCHS[_SWITCH_NUM]={
	[SWITCH_ONE]={.SWITCH_Port=GPIO_B, .SWITCH_Pin=GPIO_PIN0,  .SWITCH_Mode=SWITCH_INPUT_PU},
	[SWITCH_TWO]={.SWITCH_Port=GPIO_B, .SWITCH_Pin=GPIO_PIN1,  .SWITCH_Mode=SWITCH_INPUT_PU}
};
