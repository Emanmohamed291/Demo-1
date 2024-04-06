/*
 * LED_cfg.c
 *
 * Created: 2/18/2024 11:26:36 AM
 *  Author: Dell
 */ 

#include "GPIO.h"
#include "LED.h"

/*** LEDS user configuration ***/
const LED_cfg_t LEDS[_LED_NUM]={
	[LED_RED]={.LED_Port=GPIO_A, .LED_Pin=GPIO_PIN0, .LED_Conn=LED_CONN_FWD, .LED_Mode=output_PP, .LED_Speed=High_speed},
	[LED_GREEN]={.LED_Port=GPIO_A, .LED_Pin=GPIO_PIN1, .LED_Conn=LED_CONN_FWD, .LED_Mode=output_PP, .LED_Speed=High_speed}	
};
