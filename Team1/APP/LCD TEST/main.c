#include "RCC.h"
#include "LCD.h"
#include "LED.h"
#include "scheduler.h"

void initcbf(void)
{
	LED_SetStatus(LED_GREEN, LED_STATE_ON);
}

int main(void)
{
	RCC_enuEnableDisablePeripheral(RCC_AHB1,GPIOAEN,Periph_enuON);
	RCC_enuEnableDisablePeripheral(RCC_AHB1,GPIOBEN,Periph_enuON);
	LED_Init();
	LCD_InitPins();
	LCD_init_asynch(initcbf);
	scheduler_init();
	scheduler_start();
    /* Loop forever */
	while(1);
	return 0;
}


