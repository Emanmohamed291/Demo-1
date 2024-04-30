#include "MCAL/00_RCC/RCC.h"
#include "MCAL/01_GPIO/GPIO.h"
#include "MCAL/04_USART/USART.h"
#include "MCAL/02_NVIC/NVIC.h"
#include "STD_LIB/std_types.h"

#define USART1_TxPORT        GPIO_A
#define USART1_RxPORT        GPIO_A
#define USART1_TxPIN         GPIO_PIN9
#define USART1_RxPIN         GPIO_PIN10


void IPC_Init(u8 UART_chanel);

void IPC_SendUSART(u8 USART_Num, u8*buffer, u16 len, fnpointer cbf);

void IPC_ReceiveUSART(u8 USART_Num,u8*buffer, u16 len, fnpointer cbf);
