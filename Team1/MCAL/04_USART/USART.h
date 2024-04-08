/*
 * USART.h
 *
 * Created: 22/3/2024 5:53:43 PM
 *  Author: Eman
 */

#ifndef USART_H_
#define USART_H_

/************************************************************************************
 *                                       datatypes                                  *
 * **********************************************************************************/
#include "STD_LIB/std_types.h"
typedef enum
{
    USART_Ok,
    USART_WRONGCHANNEL,
    USART_NULLPOINTER,
    USART_wrongchannel,
    USART_WRONGFREQ,
    USART_TIMEOUT,
    USART_BUSY
} USART_ErrorStatus_t;

typedef struct
{
    u32 Channel;
    u32 Oversampling_mode;
    u32 USART_enable;
    u32 Word_length;
    u32 Wakeup_method;
    u32 Parity_control_enable;
    u32 Parity_selection;
    u32 PE_interrupt_enable;
    u32 TXE_interrupt_enable;
    u32 Transmission_complete_interrupt_enable;
    u32 RXNE_interrupt_enable;
    u32 IDLE_interrupt_enable;
    u32 Transmitter_enable;
    u32 Receiver_enable;
    u32 Receiver_wakeup;
    u32 Send_break;
    u32 STOP_bits;
    u16 baudrate;

} USART_Config_t;

typedef enum
{
    USART1_channel,
    USART2_channel,
    USART6_channel

} USART_Channels_t;

typedef struct
{
    u8 *data;
    u32 pos;
    u32 size;
} USART_buffer_t;

typedef void (*TxCB)(void);
typedef void (*RxCB)(void);

typedef struct
{
    volatile USART_buffer_t *buffer;
    volatile u32 state;
    volatile TxCB cb;
} TXReq_t;

typedef struct
{
    volatile USART_buffer_t *buffer;
    volatile u32 state;
    volatile RxCB cb;
} RXReq_t;

/************************************************************************************
 *                                       #defines                                  *
 * **********************************************************************************/

#define F_CLK 16000000
#define NUMBER_OF_USART 3
#define NULL         ((void *) 0)

#define USART_ENABLE 0x00002000
#define USART_DISABLE 0xFFFFDFFF

#define USART_OVERSAMPLING_16 0x00000000
#define USART_OVERSAMPLING_8 0x00008000

#define USART_DATA_BITS_8 0x00000000
#define USART_DATA_BITS_9 0x00001000

#define USART_PARITY_ON 0x00000400
#define USART_PARITY_OFF 0xFFFFFBFF

#define USART_PARITY_NONE 0x00000000
#define USART_PARITY_ODD 0x00000200
#define USART_PARITY_EVEN 0x00000000

#define USART_TXE_INTERRUPT_ENABLE 0x00000080
#define USART_TXE_INTERRUPT_DISABLE 0xFFFFFF7F

#define USART_TX_COMPLETE_INT_ENABLE 0x00000040
#define USART_TX_COMPLETE_INT_DISABLE 0x00000000

#define USART_RXNE_INT_ENABLE 0x00000020
#define USART_RXNE_INT_DISABLE 0xFFFFFFDF

#define USART_STOP_BITS_HALF 0x00001000
#define USART_STOP_BITS_ONE 0x00000000
#define USART_STOP_BITS_ONE_AND_HALF 0x00003000
#define USART_STOP_BITS_TWO 0x00002000

#define USART_TX_ENABLE 0x00000008
#define USART_RX_ENABLE 0x00000004

#define ENABLE           1
#define DISABLE          0

#define USART_NOT_FINISH 0
#define USART_FINISH 1

// u8 USART1_Finish_Receiving = USART_NOT_FINISH;
// u8 USART2_Finish_Receiving = USART_NOT_FINISH;
// u8 USART6_Finish_Receiving = USART_NOT_FINISH;

// typedef void(*CallBackfun)(void);
/************************************************************************************
 *                                       functions                                  *
 * **********************************************************************************/
/*
USART_ErrorStatus_t USART_Init(const USART_Config_t* ConfigPtr);
USART_ErrorStatus_t USART_SendByteAsynch(USART_Channels_t Channel, u8* buffer, u32 length, TxCB cb);
USART_ErrorStatus_t USART_ReceiveByteAsynch(USART_Channels_t Channel, u8* buffer, u32 length, RxCB cb);
USART_ErrorStatus_t USART_SendBufferZeroCopy(USART_Channels_t Channel, u8* buffer, u32 length);
*/
USART_ErrorStatus_t USART_InitAsyn(USART_Config_t *ConfigPtr);
USART_ErrorStatus_t USART_TxBufferAsyncZeroCopy(USART_Channels_t Channel, u8 *buffer, u16 len, TxCB cbf);
USART_ErrorStatus_t USART_RxBufferAsyncZeroCopy(USART_Channels_t Channel, u8 *buffer, u16 len, RxCB cbf);
USART_ErrorStatus_t USART_SendByte(USART_Channels_t Channel, u8 buffer);
USART_ErrorStatus_t USART_ReceiveByte(USART_Channels_t Channel, u8* buffer);
#endif /* USART_H_ */
