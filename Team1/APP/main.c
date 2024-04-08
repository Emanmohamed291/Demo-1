
#include "MCAL/RCC/RCC.h"
#include "MCAL/GPIO/GPIO.h"
#include "HAL/LED/LED.h"
#include "HAL/SWITCH/SWITCH.h"
/****************************************** switch **********************************/
// int main(void)
// {
// 	u8 sw1_state = SWITCH_NOT_PRESSED;
//     u8 sw2_state = SWITCH_NOT_PRESSED;

// 	RCC_enuEnableDisablePeripheral(RCC_AHB1,GPIOBEN,Periph_enuON);
// 	RCC_enuEnableDisablePeripheral(RCC_AHB1,GPIOAEN,Periph_enuON);

// 	 LED_Init();
// 	 SWITCH_Init();
//    while(1)
//    {
// 	SWITCH_GetStatus(SWITCH_ONE, &sw1_state);
//     SWITCH_GetStatus(SWITCH_TWO, &sw2_state);
// 	if(sw1_state==SWITCH_PRESSED)
// 	{
// 		LED_SetStatus(LED_RED, LED_STATE_ON);
// 		LED_SetStatus(LED_GREEN, LED_STATE_OFF);
// 	}
// 	if(sw2_state==SWITCH_PRESSED)
// 	{
// 		LED_SetStatus(LED_RED, LED_STATE_OFF);
// 		LED_SetStatus(LED_GREEN, LED_STATE_ON);
// 	}

//    }


// }
/*********************************************** lcd ********************************************/
#include "LCD.h"
#include"SERVICE/schedular/Scheduler.h"

// void initcbf(void)
// {
// 	LED_SetStatus(LED_GREEN, LED_STATE_ON);
// }

// int main(void)
// {
// 	RCC_enuEnableDisablePeripheral(RCC_AHB1,GPIOAEN,Periph_enuON);
// 	RCC_enuEnableDisablePeripheral(RCC_AHB1,GPIOBEN,Periph_enuON);
// 	LED_Init();
// 	LCD_InitPins();
// 	LCD_init_asynch(initcbf);
// 	Sched_Init();
// 	Sched_Start();
//     /* Loop forever */
// 	while(1);
// 	return 0;
// }
/******************************************** usart ******************************************/
#include "MCAL/04_USART/USART.h"
#include "MCAL/02_NVIC/NVIC.h"
u8* buffer="fadl mohamed abdelfattah ";
void task(void){
	USART_TxBufferAsyncZeroCopy(USART1_channel, buffer, 25, NULL);
}

int main()
{
	RCC_enuEnableDisablePeripheral(RCC_AHB1,GPIOAEN,Periph_enuON);
	RCC_enuEnableDisablePeripheral(RCC_AHB1,GPIOAEN,Periph_enuON);
	RCC_enuEnableDisablePeripheral(RCC_APB2, USART1EN, Periph_enuON);
	// RCC_EnablePeripheralClock(BusAHB1, GPIO_A);
	// RCC_EnablePeripheralClock(BusAHB1, GPIO_B);
	// RCC_EnablePeripheralClock(BusAHB1, GPIO_C);
	// RCC_EnablePeripheralClock(BusAPB2, USART1EN);

	NVIC_EnableInterrupt(USART1_IRQn);
 
    
	GPIO_PinCfg_t TX;
	GPIO_PinCfg_t RX;

	TX.GPIO_AF  = AF7;
	TX.GPIO_MODE  = AF_PP;
	TX.GPIO_PIN	  = GPIO_PIN9;
	TX.GPIO_PORT	= GPIO_A;
	TX.GPIO_SPEED	= High_speed;

	RX.GPIO_AF  = AF7;
	RX.GPIO_MODE  = AF_PP_PU;
	RX.GPIO_PIN	  = GPIO_PIN10;
	RX.GPIO_PORT	= GPIO_A;
	RX.GPIO_SPEED	= High_speed;

	USART_Config_t usart_config = {
        .Channel = USART1_channel,
        .Oversampling_mode = USART_OVERSAMPLING_8,
        .USART_enable = USART_ENABLE,
        .Word_length = USART_DATA_BITS_8,
        .Parity_selection = USART_PARITY_NONE,
        .Transmission_complete_interrupt_enable = USART_TX_COMPLETE_INT_ENABLE,
        .Transmitter_enable = USART_TX_ENABLE,
        .Receiver_enable = USART_RX_ENABLE,
        .STOP_bits = USART_STOP_BITS_ONE,
        .RXNE_interrupt_enable = ENABLE,
        .baudrate = 9600
    };



	GPIO_InitPin(&TX);
	GPIO_InitPin(&RX);
	//led_init();
	USART_InitAsyn(&usart_config);
	task();
// 	buffer="eman ";
//    USART_TxBufferAsyncZeroCopy(USART1_channel, buffer, 4, NULL);
	// STK_SetCallback(&task);
	// STK_Start();
	// u8 bufferbyte ='y';
	// u8* bufferby ='j';;
	// USART_SendByte(USART1_channel, bufferbyte);
	// USART_ReceiveByte(USART1_channel, bufferby);
	// USART_SendByte(USART1_channel, *bufferby);
	// USART_SendByte(USART1_channel, USART_ReceiveByte(USART1_channel, bufferby));
	// USART_SendByte(USART1_channel, bufferbyte);
	// while (1)
	// {
		
		// USART_ReceiveByte(USART1_channel, bufferby);
		// if(bufferby =="y")
        // {
		// 	u8 BUF = "H";
        //     USART_SendByte(USART1_channel, BUF);
        // }
        // else if(bufferby =="n")
        // {
		// 	u8 BUF = "B";
        //     USART_SendByte(USART1_channel, BUF);

       // }
		// USART_RxBufferAsyncZeroCopy(USART1_channel,buffer,1,NULL);
        // if(buffer[0] =='y')
        // {
		// 	u8* BUF = "HELLO";
        //     USART_TxBufferAsyncZeroCopy(USART1_channel,BUF,5,NULL);
        // }
        // else if(buffer[0] =='n')
        // {
		// 	u8* BUF = "BYE";
        //     USART_TxBufferAsyncZeroCopy(USART1_channel,BUF,3,NULL);

        // }
	
	// }
	
}