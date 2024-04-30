 //#include "STM32F4xx.h"  // Adjust according to your specific STM32 series
 #include "APP/APP_Demo1/APP_Demo.h"
#include <string.h>  // For string operations

// Define the buffer and size as global variables
#define RECEIVE_BUFFER_SIZE 256  // The size of the receive buffer
u8 receiveBuffer[RECEIVE_BUFFER_SIZE]={0};  // Buffer for incoming USART data

void USART_ReceiveHandler(void);  // Updated function signature

int main(void) {

     RCC_enuEnableDisablePeripheral(RCC_AHB1,GPIOAEN,Periph_enuON);
	IPC_Init(USART_CH1);
    LED_Init();
	SWITCH_Init();
    // Initialize USART and set up the reception callback
   
   
    // Main loop
    while (1) {
        // Continue with other tasks or idling
          IPC_ReceiveUSART(USART_CH1, receiveBuffer, RECEIVE_BUFFER_SIZE, USART_ReceiveHandler);
    }

    return 0;
}

void USART_ReceiveHandler(void) {
   
     u16 len = strlen((char*)receiveBuffer);
    char command[RECEIVE_BUFFER_SIZE]={0};
    strncpy(command, (char*)receiveBuffer, len);
    command[len] = '\0';
  
    // Handle "SET_LED_ON" command
        if (strcmp(command, "SET_LED_ON") == 0) {
            LED_SetStatus(LED_RED, LED_STATE_ON);  // Turn on LED
            IPC_SendUSART(USART_CH1, (u8*)"OK", 2, NULLPTR);  // Send confirmation
        }
        // Handle "SET_LED_OFF" command
        else if (strcmp(command, "SET_LED_OFF") == 0) {
            LED_SetStatus(LED_RED, LED_STATE_OFF);  // Turn off LED
            IPC_SendUSART(USART_CH1, (u8*)"OK", 2, NULLPTR);  // Send confirmation
        }
        // Handle "GET_SWITCH_STATUS" command
        else if (strcmp(command, "GET_SWITCH_STATUS") == 0) {
            u8 switch_status = 0;  // Store switch status
            SWITCH_GetStatus(SWITCH_UP, &switch_status);  // Get switch status, pass correct switch identifier
            
            if (switch_status == SWITCH_PRESSED) {  // If switch is pressed
                IPC_SendUSART(USART_CH1, (u8*)"PRESSED", 7, NULLPTR);
            } else {  // If switch is not pressed
                IPC_SendUSART(USART_CH1, (u8*)"RELEASED", 8, NULLPTR);
            }
        } else {
            IPC_SendUSART(USART_CH1, (u8*)"ERROR", 5, NULLPTR);  // Unknown command
        }
        // Clear the receive buffer after processing
        memset(receiveBuffer, 0, RECEIVE_BUFFER_SIZE);
    
}
