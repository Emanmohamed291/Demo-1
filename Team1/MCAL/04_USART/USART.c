/*
 * USART.c
 *
 * Created: 22/3/2024 12:58:15 AM
 *  Author: Eman
 */

#include "USART.h"
#include "STD_Types.h"
/************************************************************************************
 *                                       datatypes                                  *
 * **********************************************************************************/

typedef struct
{
    volatile u32 USART_SR;
    volatile u32 USART_DR;
    volatile u32 USART_BRR;
    volatile u32 USART_CR1;
    volatile u32 USART_CR2;
    volatile u32 USART_CR3;
    volatile u32 USART_GTPR;

} USART_Registers_t;

typedef struct
{
    u8 parity_error;
    u8 transmission_complete;
    u8 transmit_empty;
    u8 read_empty;
} interrupt_status_t;

interrupt_status_t interrupt[NUMBER_OF_USART];

enum
{
    SBK,
    RWU,
    RE,
    TE,
    IDLEIE,
    RXNEIE,
    TCIE,
    TXEIE,
    PEIE,
    PS,
    PCE,
    WAKE,
    M,
    UE,
    OVER8 = 15
} USART_ctrl_pins;

enum
{
    PE,
    FE,
    NF,
    ORE,
    IDLE,
    RXNE,
    TC,
    TXE,
    LBD,
    CTS
} USART_SR_pins;
enum
{
    oversampling_by_16,
    oversampling_by_8
};
/************************************************************************************
 *                                       #defines                                  *
 * **********************************************************************************/
#define USART1 ((USART_Registers_t *)0x40011000)
#define USART2 ((USART_Registers_t *)0x40004400)
#define USART6 ((USART_Registers_t *)0x40011400)

//#define NULL ((void *)0)

#define USART_MASK_TXEIE_ENABLE 0X00000080
#define USART_MASK_TXEIE_DISABLE 0XFFFFFF7F
#define USART_MASK_RXNEIE_ENABLE 0X00000020
#define USART_MASK_RXNEIE_DISABLE 0XFFFFFFDF
#define USART_MASK_CLEAR_TC 0xFFFFFFBF
#define USART_DISABLE_PARITY 0xFFFFFBFF
// #define USART1_READY_BIT                            0b00000001
#define USART_TXE_FLAG_MASK 0X00000080
#define USART_RXNE_FLAG_MASK 0X00000020

#define MANTISA_OFFSET 4

#define USART_Busy 1
#define USART_Ready 0
/************************************************************************************
 *                                       variables                                  *
 * **********************************************************************************/
/* variable to save interrupt status which is enable and disable
for the config of the parameter of the init function*/
// static interrupt_status interrupt[NUMBER_OF_USART];
USART_buffer_t volatile usart_buffer;
USART_buffer_t volatile usart_buffer_rx;
TXReq_t volatile USART_TXReq[NUMBER_OF_USART] = {
    {.state = USART_Ready},
    {.state = USART_Ready},
    {.state = USART_Ready},
};
RXReq_t volatile USART_RXReq[NUMBER_OF_USART] = {
    {.state = USART_Ready},
    {.state = USART_Ready},
    {.state = USART_Ready},
};
/************************************************************************************
 *                                       functions                                  *
 * **********************************************************************************/
USART_ErrorStatus_t USART_InitAsyn(USART_Config_t *ConfigPtr)
{
    USART_ErrorStatus_t ErrorStatusLocVar = USART_Ok;
    if (ConfigPtr == NULL)
    {
        ErrorStatusLocVar = USART_NULLPOINTER;
    }
    else if (ConfigPtr->Channel > USART6_channel)
    {
        ErrorStatusLocVar = USART_WRONGCHANNEL;
    }
    else
    {
        /* DIV and Fracion */
        f32 Loc_OVER8or16 = 0;
        switch (ConfigPtr->Oversampling_mode)
        {
        case USART_OVERSAMPLING_16:
            Loc_OVER8or16 = oversampling_by_16;
            break;
        case USART_OVERSAMPLING_8:
            Loc_OVER8or16 = oversampling_by_8;
            break;
        }
        f32 USARTDIV = ((f32)(F_CLK) / ((ConfigPtr->baudrate) * 8 * (2 - Loc_OVER8or16)));
        f32 FracionBoundary = (ConfigPtr->Oversampling_mode == USART_OVERSAMPLING_16) ? 16 : 8;
        /* need to multiply the fraction by 8 or 16 depends on the chosen oversample */
        u32 DIV_Fraction = (u32)(FracionBoundary * (f32)((f32)USARTDIV - (u32)USARTDIV));
        u32 MAXVALUE = (ConfigPtr->Oversampling_mode == USART_OVERSAMPLING_16) ? 15 : 7;
        u32 DIV_Mantissa = 0;
        if (DIV_Fraction > MAXVALUE)
        {
            DIV_Fraction = 0;
            DIV_Mantissa = (u32)USARTDIV++;
        }
        else
        {
            DIV_Mantissa = (u32)USARTDIV;
        }
        switch (ConfigPtr->Channel)
        {
        case USART1_channel:
            /* baudrate reg */
            USART1->USART_BRR = (DIV_Mantissa << MANTISA_OFFSET) | (DIV_Fraction);
            /* because usart is off so i can use reg direct not in need to temp var
               because i use struct so i didn't use macros u32 because struct size will be over*/
            USART1->USART_CR1 = 0;
            USART1->USART_CR1 |= (ConfigPtr->Oversampling_mode);
            USART1->USART_CR1 |= (ConfigPtr->Word_length);
            USART1->USART_CR1 |= (ConfigPtr->Wakeup_method);
            USART1->USART_CR1 |= (ConfigPtr->Transmitter_enable);
            USART1->USART_CR1 |= (ConfigPtr->Receiver_enable);
            /* parity */
            if (ConfigPtr->Parity_selection == USART_PARITY_NONE)
            {
                USART1->USART_CR1 &= (USART_PARITY_OFF);
            }
            else
            {
                USART1->USART_CR1 |= (ConfigPtr->Parity_control_enable);
                USART1->USART_CR1 |= (ConfigPtr->Parity_selection);
            }

            USART1->USART_CR2 |= (ConfigPtr->STOP_bits);

            /* clear status reg */
            USART1->USART_SR = 0;
            /* enable USART1 */
            USART1->USART_CR1 |= (ConfigPtr->USART_enable);
            break;
        case USART2_channel:
            /* baudrate reg */
            USART2->USART_BRR = (DIV_Mantissa << MANTISA_OFFSET) | (DIV_Fraction);
            /* because usart is off so i can use reg direct not in need to temp var
               because i use struct so i didn't use macros u32 because struct size will be over*/
            USART2->USART_CR1 = 0;
            USART2->USART_CR1 |= (ConfigPtr->Oversampling_mode);
            USART2->USART_CR1 |= (ConfigPtr->Word_length);
            USART2->USART_CR1 |= (ConfigPtr->Wakeup_method);
            USART2->USART_CR1 |= (ConfigPtr->Transmitter_enable);
            USART2->USART_CR1 |= (ConfigPtr->Receiver_enable);
            /* parity */
            if (ConfigPtr->Parity_selection == USART_PARITY_NONE)
            {
                USART2->USART_CR1 &= (USART_PARITY_OFF);
            }
            else
            {
                USART2->USART_CR1 |= (ConfigPtr->Parity_control_enable);
                USART2->USART_CR1 |= (ConfigPtr->Parity_selection);
            }

            USART2->USART_CR2 |= (ConfigPtr->STOP_bits);

            /* clear status reg */
            USART2->USART_SR = 0;
            /* enable USART2 */
            USART2->USART_CR1 |= (ConfigPtr->USART_enable);
            break;
        case USART6_channel:
            /* baudrate reg */
            USART6->USART_BRR = (DIV_Mantissa << MANTISA_OFFSET) | (DIV_Fraction);
            /* because usart is off so i can use reg direct not in need to temp var
               because i use struct so i didn't use macros u32 because struct size will be over*/
            USART6->USART_CR1 = 0;
            USART6->USART_CR1 |= (ConfigPtr->Oversampling_mode);
            USART6->USART_CR1 |= (ConfigPtr->Word_length);
            USART6->USART_CR1 |= (ConfigPtr->Wakeup_method);
            USART6->USART_CR1 |= (ConfigPtr->Transmitter_enable);
            USART6->USART_CR1 |= (ConfigPtr->Receiver_enable);
            /* parity */
            if (ConfigPtr->Parity_selection == USART_PARITY_NONE)
            {
                USART6->USART_CR1 &= (USART_PARITY_OFF);
            }
            else
            {
                USART6->USART_CR1 |= (ConfigPtr->Parity_control_enable);
                USART6->USART_CR1 |= (ConfigPtr->Parity_selection);
            }

            USART6->USART_CR2 |= (ConfigPtr->STOP_bits);

            /* clear status reg */
            USART6->USART_SR = 0;

            USART6->USART_CR1 |= (ConfigPtr->USART_enable);
            break;
        }
    }
    return ErrorStatusLocVar;
}
/************************************************************************************************************************/
USART_ErrorStatus_t USART_TxBufferAsyncZeroCopy(USART_Channels_t Channel, u8 *buffer, u16 len, TxCB cbf)
{
    USART_ErrorStatus_t ErrorStatusLocVar = USART_Ok;
    if (buffer == NULL)
    {
        ErrorStatusLocVar = USART_NULLPOINTER;
    }
    else if ((Channel != USART1_channel) && (Channel != USART2_channel) && (Channel != USART6_channel))
    {
        ErrorStatusLocVar = USART_WRONGCHANNEL;
    }
    else
    {
        if ((USART_TXReq[Channel].state) == USART_Ready)
        {
            switch (Channel)
            {
            case USART1_channel:
                USART_TXReq[Channel].state = USART_Busy;

                USART1->USART_CR1 |= USART_TXE_INTERRUPT_ENABLE;

                usart_buffer.data = buffer;
                usart_buffer.size = len;
                usart_buffer.pos = 0;

                // TXReq_t USART_TXReq;
                USART_TXReq[Channel].buffer = &usart_buffer;

                // USART_TXReq[Channel].buffer->data = buffer;
                // USART_TXReq[Channel].buffer->size = len;
                USART_TXReq[Channel].cb = cbf;
                // USART_TXReq[Channel].buffer->pos = 0;
                USART_TXReq[Channel].state = USART_Busy;

                USART1->USART_DR = USART_TXReq[Channel].buffer->data[0];
                USART_TXReq[Channel].buffer->pos++;
                break;
            case USART2_channel:
                USART_TXReq[Channel].state = USART_Busy;

                USART2->USART_CR1 |= USART_TXE_INTERRUPT_ENABLE;

                // USART_buffer_t usart_buffer;
                usart_buffer.data = buffer;
                usart_buffer.size = len;
                usart_buffer.pos = 0;

                USART_TXReq[Channel].buffer = &usart_buffer;

                // USART_TXReq[Channel].buffer->data = buffer;
                // USART_TXReq[Channel].buffer->size = len;
                USART_TXReq[Channel].cb = cbf;
                // USART_TXReq[Channel].buffer->pos = 0;
                USART_TXReq[Channel].state = USART_Busy;

                USART2->USART_DR = USART_TXReq[Channel].buffer->data[0];
                USART_TXReq[Channel].buffer->pos++;
                break;
            case USART6_channel:
                USART_TXReq[Channel].state = USART_Busy;

                // USART_buffer_t usart_buffer;
                usart_buffer.data = buffer;
                usart_buffer.size = len;
                usart_buffer.pos = 0;

                USART_TXReq[Channel].buffer = &usart_buffer;

                // USART_TXReq[Channel].buffer->data = buffer;
                // USART_TXReq[Channel].buffer->size = len;
                USART_TXReq[Channel].cb = cbf;
                // USART_TXReq[Channel].buffer->pos = 0;
                USART_TXReq[Channel].state = USART_Busy;

                USART6->USART_DR = USART_TXReq[Channel].buffer->data[0];
                USART_TXReq[Channel].buffer->pos++;

                USART6->USART_CR1 |= USART_TXE_INTERRUPT_ENABLE;
                break;
            }
        }
        else
        {
            ErrorStatusLocVar = USART_BUSY;
        }
    }

    return ErrorStatusLocVar;
}
/************************************************************************************************************************/
USART_ErrorStatus_t USART_RxBufferAsyncZeroCopy(USART_Channels_t Channel, u8 *buffer, u16 len, RxCB cbf)
{
    USART_ErrorStatus_t ErrorStatusLocVar = USART_Ok;
    if (buffer == NULL)
    {
        ErrorStatusLocVar = USART_NULLPOINTER;
    }
    else if ((Channel != USART1_channel) && (Channel != USART2_channel) && (Channel != USART6_channel))
    {
        ErrorStatusLocVar = USART_WRONGCHANNEL;
    }
    else
    {
        if ((USART_RXReq[Channel].state) == USART_Ready)
        {
            // USART_buffer_t usart_buffer;
            switch (Channel)
            {
            case USART1_channel:
                USART_RXReq[Channel].state = USART_Busy;

                USART1->USART_CR1 &= USART_RXNE_INT_DISABLE;

                usart_buffer_rx.data = buffer;
                usart_buffer_rx.size = len;
                usart_buffer_rx.pos = 0;

                // TXReq_t USART_TXReq;
                USART_RXReq[Channel].buffer = &usart_buffer_rx;

                // USART_TXReq[Channel].buffer->data = buffer;
                // USART_TXReq[Channel].buffer->size = len;
                USART_RXReq[Channel].cb = cbf;
                // USART_TXReq[Channel].buffer->pos = 0;
                USART_RXReq[Channel].state = USART_Busy;

                USART1->USART_CR1 |= USART_RXNE_INT_ENABLE;
                break;
            case USART2_channel:
                USART_RXReq[Channel].state = USART_Busy;

                USART2->USART_CR1 &= USART_RXNE_INT_DISABLE;

                // USART_buffer_t usart_buffer;
                usart_buffer_rx.data = buffer;
                usart_buffer_rx.size = len;
                usart_buffer_rx.pos = 0;

                // TXReq_t USART_TXReq;
                USART_RXReq[Channel].buffer = &usart_buffer_rx;

                // USART_TXReq[Channel].buffer->data = buffer;
                // USART_TXReq[Channel].buffer->size = len;
                USART_RXReq[Channel].cb = cbf;
                // USART_TXReq[Channel].buffer->pos = 0;
                USART_RXReq[Channel].state = USART_Busy;

                USART2->USART_CR1 |= USART_RXNE_INT_ENABLE;
                break;
            case USART6_channel:
                USART_RXReq[Channel].state = USART_Busy;

                USART6->USART_CR1 &= USART_RXNE_INT_DISABLE;

                // USART_buffer_t usart_buffer;
                usart_buffer_rx.data = buffer;
                usart_buffer_rx.size = len;
                usart_buffer_rx.pos = 0;

                // TXReq_t USART_TXReq;
                USART_RXReq[Channel].buffer = &usart_buffer_rx;

                // USART_TXReq[Channel].buffer->data = buffer;
                // USART_TXReq[Channel].buffer->size = len;
                USART_RXReq[Channel].cb = cbf;
                // USART_TXReq[Channel].buffer->pos = 0;
                USART_RXReq[Channel].state = USART_Busy;

                USART6->USART_CR1 |= USART_RXNE_INT_ENABLE;
                break;
            }
        }
        else
        {
            ErrorStatusLocVar = USART_BUSY;
        }
    }

    return ErrorStatusLocVar;
}

/***************************************************************************************************************/
USART_ErrorStatus_t USART_SendByte(USART_Channels_t Channel, u8 buffer)
{
    USART_ErrorStatus_t ErrorStatusLocVar = USART_Ok;
    if ((Channel != USART1_channel) && (Channel != USART2_channel) && (Channel != USART6_channel))
    {
        ErrorStatusLocVar = USART_WRONGCHANNEL;
    }
    else
    {
        u32 Timeout = 16000;
        switch (Channel)
        {
        case USART1_channel:
            if (USART_TXReq[Channel].state == USART_Ready)
            {
                USART_TXReq[Channel].state = USART_Busy;
                USART1->USART_DR = buffer;
                while (Timeout-- && (!(USART1->USART_SR & USART_TXE_FLAG_MASK)))
                    ;
                USART_TXReq[Channel].state = USART_Ready;
                if (!(USART1->USART_SR & USART_TXE_FLAG_MASK))
                {
                    ErrorStatusLocVar = USART_TIMEOUT;
                }
            }
            else
            {
                ErrorStatusLocVar = USART_BUSY;
            }
            break;
        case USART2_channel:
            if (USART_TXReq[Channel].state == USART_Ready)
            {
                USART_TXReq[Channel].state = USART_Busy;
                USART2->USART_DR = buffer;
                while (Timeout-- && (!(USART2->USART_SR & USART_TXE_FLAG_MASK)))
                    ;
                USART_TXReq[Channel].state = USART_Ready;
                if (!(USART2->USART_SR & USART_TXE_FLAG_MASK))
                {
                    ErrorStatusLocVar = USART_TIMEOUT;
                }
            }
            else
            {
                ErrorStatusLocVar = USART_BUSY;
            }
            break;
        case USART6_channel:
            if (USART_TXReq[Channel].state == USART_Ready)
            {
                USART_TXReq[Channel].state = USART_Busy;
                USART6->USART_DR = buffer;
                while (Timeout-- && (!(USART6->USART_SR & USART_TXE_FLAG_MASK)))
                    ;
                USART_TXReq[Channel].state = USART_Ready;
                if (!(USART6->USART_SR & USART_TXE_FLAG_MASK))
                {
                    ErrorStatusLocVar = USART_TIMEOUT;
                }
            }
            else
            {
                ErrorStatusLocVar = USART_BUSY;
            }
            break;
        default:
            break;
        }
    }
    return ErrorStatusLocVar;
}
/***************************************************************************************************************/
USART_ErrorStatus_t USART_ReceiveByte(USART_Channels_t Channel, u8 *buffer)
{
    USART_ErrorStatus_t ErrorStatusLocVar = USART_Ok;
    if (buffer == NULL)
    {
        ErrorStatusLocVar = USART_NULLPOINTER;
    }
    else if ((Channel != USART1_channel) && (Channel != USART2_channel) && (Channel != USART6_channel))
    {
        ErrorStatusLocVar = USART_WRONGCHANNEL;
    }
    else
    {
        u32 Timeout = 16000;
        switch (Channel)
        {
        case USART1_channel:
            if (USART_RXReq[Channel].state == USART_Ready)
            {
                USART_RXReq[Channel].state = USART_Busy;
                /* if timeout=0 or RXNE = 1 then it received */
                while (Timeout-- && (!(USART1->USART_SR & USART_RXNE_FLAG_MASK)))
                    ;
                USART_TXReq[Channel].state = USART_Ready;
                if (USART1->USART_SR & USART_TXE_FLAG_MASK)
                {
                    *buffer = USART1->USART_DR;
                }
                else
                {
                    ErrorStatusLocVar = USART_TIMEOUT;
                }
            }
            else
            {
                ErrorStatusLocVar = USART_BUSY;
            }
            break;
        case USART2_channel:
            if (USART_RXReq[Channel].state == USART_Ready)
            {
                USART_RXReq[Channel].state = USART_Busy;
                /* if timeout=0 or RXNE = 1 then it received */
                while (Timeout-- && (!(USART2->USART_SR & USART_RXNE_FLAG_MASK)))
                    ;
                USART_TXReq[Channel].state = USART_Ready;
                if (USART2->USART_SR & USART_TXE_FLAG_MASK)
                {
                    *buffer = USART2->USART_DR;
                }
                else
                {
                    ErrorStatusLocVar = USART_TIMEOUT;
                }
            }
            else
            {
                ErrorStatusLocVar = USART_BUSY;
            }
            break;
        case USART6_channel:
            if (USART_RXReq[Channel].state == USART_Ready)
            {
                USART_RXReq[Channel].state = USART_Busy;
                /* if timeout=0 or RXNE = 1 then it received */
                while (Timeout-- && (!(USART6->USART_SR & USART_RXNE_FLAG_MASK)))
                    ;
                USART_TXReq[Channel].state = USART_Ready;
                if (USART6->USART_SR & USART_TXE_FLAG_MASK)
                {
                    *buffer = USART6->USART_DR;
                }
                else
                {
                    ErrorStatusLocVar = USART_TIMEOUT;
                }
            }
            else
            {
                ErrorStatusLocVar = USART_BUSY;
            }
            break;
        default:
            break;
        }
    }
    return ErrorStatusLocVar;
}
/***************************************************************************************************************/
void USART1_IRQHandler(void)
{
    /* If tx flag is fired */
    if ((USART1->USART_SR & USART_TXE_FLAG_MASK))
    {
        if (USART_TXReq[USART1_channel].buffer->pos < USART_TXReq[USART1_channel].buffer->size)
        {
            USART1->USART_SR &= ~(1 << 6);
            USART1->USART_DR = USART_TXReq[USART1_channel].buffer->data[USART_TXReq[USART1_channel].buffer->pos++];
        }
        else
        {
            USART_TXReq[USART1_channel].state = USART_Ready;
            USART_TXReq[USART1_channel].buffer->pos = 0;

            USART1->USART_SR &= ~(1 << 6);
            USART1->USART_CR1 &= USART_TXE_INTERRUPT_DISABLE;

            if (USART_TXReq[USART1_channel].cb)
            {
                USART_TXReq[USART1_channel].cb();
            }
        }
    }

    /* If rx flag is fired */
    if (USART1->USART_SR & USART_RXNE_FLAG_MASK)
    {
        if (USART_RXReq[USART1_channel].buffer->pos < USART_RXReq[USART1_channel].buffer->size)
        {
            USART_RXReq[USART1_channel].buffer->data[USART_RXReq[USART1_channel].buffer->pos++] = USART1->USART_DR;
        }
        if (USART_RXReq[USART1_channel].buffer->pos == USART_RXReq[USART1_channel].buffer->size)
        {
            USART1->USART_SR &= ~(1 << 5);
            USART1->USART_CR1 &= USART_RXNE_INT_DISABLE;
            USART_RXReq[USART1_channel].state = USART_Ready;
            USART_RXReq[USART1_channel].buffer->pos = 0;
            if (USART_RXReq[USART1_channel].cb)
            {
                USART_RXReq[USART1_channel].cb();
            }
        }
    }
}
/***************************************************************************************************************/
void USART2_IRQHandler(void)
{
    /* If tx flag is fired */
    if ((USART2->USART_SR & USART_TXE_FLAG_MASK))
    {

        if (USART_TXReq[USART2_channel].buffer->pos < USART_TXReq[USART2_channel].buffer->size)
        {
            USART2->USART_DR = USART_TXReq[USART2_channel].buffer->data[USART_TXReq[USART2_channel].buffer->pos++];
        }
        else
        {
            USART_TXReq[USART2_channel].state = USART_Ready;
            USART_TXReq[USART2_channel].buffer->pos = 0;

            if (USART_TXReq[USART2_channel].cb)
            {
                USART_TXReq[USART2_channel].cb();
            }
        }
    }

    /* If rx flag is fired */
    if (USART2->USART_SR & USART_RXNE_FLAG_MASK)
    {
        if (USART_RXReq[USART2_channel].buffer->pos < USART_RXReq[USART2_channel].buffer->size)
        {

            USART_RXReq[USART2_channel].buffer->data[USART_RXReq[USART2_channel].buffer->pos++] = USART2->USART_DR;
        }
        else
        {
            USART_RXReq[USART2_channel].state = USART_Ready;
            USART_RXReq[USART2_channel].buffer->pos = 0;

            if (USART_RXReq[USART2_channel].cb)
            {
                USART_RXReq[USART2_channel].cb();
            }
        }
    }
}
/***************************************************************************************************************/
void USART6_IRQHandler(void)
{
    /* If tx flag is fired */
    if ((USART6->USART_SR & USART_TXE_FLAG_MASK))
    {

        if (USART_TXReq[USART6_channel].buffer->pos < USART_TXReq[USART6_channel].buffer->size)
        {
            USART6->USART_DR = USART_TXReq[USART6_channel].buffer->data[USART_TXReq[USART6_channel].buffer->pos++];
        }
        else
        {
            USART_TXReq[USART6_channel].state = USART_Ready;
            USART_TXReq[USART6_channel].buffer->pos = 0;

            if (USART_TXReq[USART6_channel].cb)
            {
                USART_TXReq[USART6_channel].cb();
            }
        }
    }

    /* If rx flag is fired */
    if (USART6->USART_SR & USART_RXNE_FLAG_MASK)
    {
        if (USART_RXReq[USART6_channel].buffer->pos < USART_RXReq[USART6_channel].buffer->size)
        {

            USART_RXReq[USART6_channel].buffer->data[USART_RXReq[USART6_channel].buffer->pos++] = USART6->USART_DR;
        }
        else
        {
            USART_RXReq[USART6_channel].state = USART_Ready;
            USART_RXReq[USART6_channel].buffer->pos = 0;

            if (USART_RXReq[USART6_channel].cb)
            {
                USART_RXReq[USART6_channel].cb();
            }
        }
    }
}
// USART_ErrorStatus_t USART_Init(const USART_Config_t* ConfigPtr){
//     USART_ErrorStatus_t ErrorStatusLocVar = USART_Ok;
//     u32 Loc_u32TempValue;
//     u16 Loc_u16TempFraction;
//     u16 Loc_u16TempMantissa;

//     if(ConfigPtr == NULL)
// 	{
// 		ErrorStatusLocVar = USART_NULLPOINTER;
// 	}
//     else if(F_CLK >84000000){
//         ErrorStatusLocVar = USART_WRONGFREQ;
//     }
//     else{
//         /* DIV and Fracion */
//         Loc_u32TempValue = ((u64)F_CLK * 1000) / (ConfigPtr->baudrate * (8 * (2 - ConfigPtr->Oversampling_mode)));
//         /* in O.S 8 so it has 3 bit fraction and in O.S 16 so it has 4 bit fraction */
//         Loc_u16TempFraction = (Loc_u32TempValue % 1000) * (8 * (2 - ConfigPtr->Oversampling_mode));
// 		Loc_u16TempFraction = Loc_u16TempFraction / 1000;
//         /* mantisa value */
//         Loc_u16TempMantissa = Loc_u32TempValue / 1000;
//         /* fraction */
//         if(ConfigPtr->Oversampling_mode ==  oversampling_by_16){
//             /* if fraction >16 which mean more than 4 bits for fraction */
//             if(Loc_u16TempFraction > 0xF)
//             {
//                 Loc_u16TempMantissa++;
//                 Loc_u16TempFraction = 0;
//             }
//         }
//         else if(ConfigPtr->Oversampling_mode ==  oversampling_by_8){
//             /* if fraction >8 which mean more than 3 bits for fraction */
//             if(Loc_u16TempFraction > 0x7)
//             {
//                 Loc_u16TempMantissa++;
//                 Loc_u16TempFraction = 0;
//             }
//         }
//         TXReq[USART1_channel].state = USART_Ready;
//         TXReq[USART2_channel].state = USART_Ready;
//         TXReq[USART6_channel].state = USART_Ready;

//         switch (ConfigPtr->Channel)
//         {
//         case USART1_channel:
//             /* baudrate reg */
// 			USART1->USART_BRR = (Loc_u16TempMantissa << 4) | (Loc_u16TempFraction & 0x0F);
//             /* because usart is off so i can use reg direct not in need to temp var
//                because i use struct so i didn't use macros u32 because struct size will be over*/
//             USART1->USART_CR1 = 0;
//             USART1->USART_CR1 |= (ConfigPtr->Oversampling_mode<<OVER8);
//             USART1->USART_CR1 |= (ConfigPtr->Word_length<<M);
//             USART1->USART_CR1 |= (ConfigPtr->Wakeup_method<<WAKE);
//             USART1->USART_CR1 |= (ConfigPtr->Parity_control_enable<<PCE);
//             if(ConfigPtr->Parity_selection<<PS == PARITY_EVEN || ConfigPtr->Parity_selection<<PS == PARITY_ODD){
//                 USART1->USART_CR1 |= (ConfigPtr->Parity_selection<<PS);
//             }
//             else{
//                 USART1->USART_CR1 &= USART_DISABLE_PARITY;
//             }
//             USART1->USART_CR1 |= (ConfigPtr->Transmitter_enable<<TE);
//             USART1->USART_CR1 |= (ConfigPtr->Receiver_enable<<RE);
//             /* interrupt status */
//             interrupt[USART1_channel].parity_error = (ConfigPtr->Receiver_enable<<PEIE);
//             interrupt[USART1_channel].read_empty = (ConfigPtr->Receiver_enable<<RXNEIE);
//             interrupt[USART1_channel].transmit_empty = (ConfigPtr->Receiver_enable<<TXEIE);
//             interrupt[USART1_channel].transmission_complete = (ConfigPtr->Receiver_enable<<TCIE);
//             /* clear status reg */
//             USART1->USART_SR = 0;
//             /* enable USART1 */
//             USART1->USART_CR1 |= (ConfigPtr->USART_enable<<UE);
//         break;
//         case USART2_channel:
//             /* baudrate reg */
// 			USART2->USART_BRR = (Loc_u16TempMantissa << 4) | (Loc_u16TempFraction & 0x0F);
//             /* because usart is off so i can use reg direct not in need to temp var
//                because i use struct so i didn't use macros u32 because struct size will be over*/
//             USART2->USART_CR1 = 0;
//             USART2->USART_CR1 |= (ConfigPtr->Oversampling_mode<<OVER8);
//             USART2->USART_CR1 |= (ConfigPtr->Word_length<<M);
//             USART2->USART_CR1 |= (ConfigPtr->Wakeup_method<<WAKE);
//             USART2->USART_CR1 |= (ConfigPtr->Parity_control_enable<<PCE);
//             if(ConfigPtr->Parity_selection<<PS == PARITY_EVEN || ConfigPtr->Parity_selection<<PS == PARITY_ODD){
//                 USART2->USART_CR1 |= (ConfigPtr->Parity_selection<<PS);
//             }
//             else{
//                 USART2->USART_CR1 &= USART_DISABLE_PARITY;
//             }
//             USART2->USART_CR1 |= (ConfigPtr->Transmitter_enable<<TE);
//             USART2->USART_CR1 |= (ConfigPtr->Receiver_enable<<RE);
//              /* interrupt status */
//             interrupt[USART2_channel].parity_error = (ConfigPtr->Receiver_enable<<PEIE);
//             interrupt[USART2_channel].read_empty = (ConfigPtr->Receiver_enable<<RXNEIE);
//             interrupt[USART2_channel].transmit_empty = (ConfigPtr->Receiver_enable<<TXEIE);
//             interrupt[USART2_channel].transmission_complete = (ConfigPtr->Receiver_enable<<TCIE);
//             /* clear status reg */
//             USART2->USART_SR = 0;
//             /* enable USART2 */
//             USART2->USART_CR1 |= (ConfigPtr->USART_enable<<UE);
//         break;
//         case USART6_channel:
//             /* baudrate reg */
// 			USART6->USART_BRR = (Loc_u16TempMantissa << 4) | (Loc_u16TempFraction & 0x0F);
//             /* because usart is off so i can use reg direct not in need to temp var
//                because i use struct so i didn't use macros u32 because struct size will be over*/
//             USART6->USART_CR1 = 0;
//             USART6->USART_CR1 |= (ConfigPtr->Oversampling_mode<<OVER8);
//             USART6->USART_CR1 |= (ConfigPtr->Word_length<<M);
//             USART6->USART_CR1 |= (ConfigPtr->Wakeup_method<<WAKE);
//             USART6->USART_CR1 |= (ConfigPtr->Parity_control_enable<<PCE);
//             if(ConfigPtr->Parity_selection<<PS == PARITY_EVEN || ConfigPtr->Parity_selection<<PS == PARITY_ODD){
//                 USART6->USART_CR1 |= (ConfigPtr->Parity_selection<<PS);
//             }
//             else{
//                 USART6->USART_CR1 &= USART_DISABLE_PARITY;
//             }
//             USART6->USART_CR1 |= (ConfigPtr->Transmitter_enable<<TE);
//             USART6->USART_CR1 |= (ConfigPtr->Receiver_enable<<RE);

//              /* interrupt status */
//             interrupt[USART6_channel].parity_error = (ConfigPtr->Receiver_enable<<PEIE);
//             interrupt[USART6_channel].read_empty = (ConfigPtr->Receiver_enable<<RXNEIE);
//             interrupt[USART6_channel].transmit_empty = (ConfigPtr->Receiver_enable<<TXEIE);
//             interrupt[USART6_channel].transmission_complete = (ConfigPtr->Receiver_enable<<TCIE);
//             /* clear status reg */
//             USART6->USART_SR = 0;
//             /* enable USART6 */
//             USART6->USART_CR1 |= (ConfigPtr->USART_enable<<UE);
//         break;
//         default:
//             ErrorStatusLocVar = USART_WRONGCHANNEL;
//         break;
//         }
//     }
//     return ErrorStatusLocVar;
// }

// /************************************************* send byte **************************************/
// USART_ErrorStatus_t USART_SendByteAsynch(USART_Channels_t Channel, u8* buffer, u32 length, TxCB cb){
//     USART_ErrorStatus_t ErrorStatusLocVar = USART_Ok;
//     if(buffer == NULL || cb== NULL ){
//         ErrorStatusLocVar = USART_NULLPOINTER;
//     }
//     else if(TXReq[Channel].state == USART_Busy){
//         ErrorStatusLocVar = USART_BUSY;
//     }
//     else{
//         switch (Channel)
//         {
//         case USART1_channel:
//             /* make TXEIE --> 1*/
//             USART1->USART_CR1 |= USART_MASK_TXEIE_ENABLE;
//             if(TXReq[USART1_channel].state == USART_Ready){
//                 TXReq[USART1_channel].state = USART_Busy;
//             }
//             else{
//                 ErrorStatusLocVar = USART_BUSY;
//             }
//             TXReq[USART1_channel].buffer.data = buffer;
//             TXReq[USART1_channel].buffer.size = length;
//             TXReq[USART1_channel].buffer.pos = 0;
//             TXReq[USART1_channel].cb = cb;
//             USART1->USART_DR = TXReq[USART1_channel].buffer.data[0];
//             TXReq[USART1_channel].buffer.pos++;
//         break;
//         case USART2_channel:
//             USART2->USART_CR1 |= USART_MASK_TXEIE_ENABLE;
//             if(TXReq[USART2_channel].state == USART_Ready){
//                 TXReq[USART2_channel].state = USART_Busy;
//             }
//             TXReq[USART2_channel].buffer.data = buffer;
//             TXReq[USART2_channel].buffer.size = length;
//             TXReq[USART2_channel].buffer.pos = 0;
//             TXReq[USART2_channel].cb = cb;
//             USART2->USART_DR = TXReq[USART2_channel].buffer.data[0];
//             TXReq[USART2_channel].buffer.pos++;
//         break;
//         case USART6_channel:
//             USART6->USART_CR1 |= USART_MASK_TXEIE_ENABLE;
//             if(TXReq[USART6_channel].state == USART_Ready){
//                 TXReq[USART6_channel].state = USART_Busy;
//             }
//             TXReq[USART6_channel].buffer.data = buffer;
//             TXReq[USART6_channel].buffer.size = length;
//             TXReq[USART6_channel].buffer.pos = 0;
//             TXReq[USART6_channel].cb = cb;
//             USART6->USART_DR = TXReq[USART6_channel].buffer.data[0];
//             TXReq[USART6_channel].buffer.pos++;
//         break;

//         default:
//             ErrorStatusLocVar = USART_wrongchannel;
//         break;
//         }
//     }

//     return ErrorStatusLocVar;
// }
// /************************************************* receive byte **************************************/
// USART_ErrorStatus_t USART_ReceiveByteAsynch(USART_Channels_t Channel, u8* buffer, u32 length, RxCB cb){
//     USART_ErrorStatus_t ErrorStatusLocVar = USART_Ok;
//     if(buffer == NULL || cb== NULL ){
//         ErrorStatusLocVar = USART_NULLPOINTER;
//     }
//     else if(RXReq[Channel].state == USART_Busy){
//         ErrorStatusLocVar = USART_BUSY;
//     }
//     else{
//         switch (Channel)
//         {
//             case USART1_channel:
//             /* make RXNEIE --> 0*/
//             USART1->USART_CR1 |= USART_MASK_RXNEIE_DISABLE;
//             if(RXReq[USART1_channel].state == USART_Ready){
//                 RXReq[USART1_channel].state = USART_Busy;
//             }
//             else{
//                 ErrorStatusLocVar = USART_BUSY;
//             }
//             RXReq[USART1_channel].buffer.data = buffer;
//             RXReq[USART1_channel].buffer.size = length;
//             RXReq[USART1_channel].buffer.pos = 0;
//             RXReq[USART1_channel].cb = cb;
//             /* make RXNEIE --> 1*/
//             USART1->USART_CR1 |= USART_MASK_RXNEIE_ENABLE;
//             break;
//             case USART2_channel:
//             /* make RXNEIE --> 0*/
//             USART2->USART_CR1 |= USART_MASK_RXNEIE_DISABLE;
//             if(RXReq[USART2_channel].state == USART_Ready){
//                 RXReq[USART2_channel].state = USART_Busy;
//             }
//             RXReq[USART2_channel].buffer.data = buffer;
//             RXReq[USART2_channel].buffer.size = length;
//             RXReq[USART2_channel].buffer.pos = 0;
//             RXReq[USART2_channel].cb = cb;
//             /* make RXNEIE --> 1*/
//             USART2->USART_CR1 |= USART_MASK_RXNEIE_ENABLE;
//             break;
//             case USART6_channel:
//             /* make RXNEIE --> 0*/
//             USART6->USART_CR1 |= USART_MASK_RXNEIE_DISABLE;
//             if(RXReq[USART6_channel].state == USART_Ready){
//                 RXReq[USART6_channel].state = USART_Busy;
//             }
//             RXReq[USART6_channel].buffer.data = buffer;
//             RXReq[USART6_channel].buffer.size = length;
//             RXReq[USART6_channel].buffer.pos = 0;
//             RXReq[USART6_channel].cb = cb;
//             /* make RXNEIE --> 1*/
//             USART6->USART_CR1 |= USART_MASK_RXNEIE_ENABLE;
//             break;

//             default:
//             ErrorStatusLocVar = USART_wrongchannel;
//             break;
//         }
//     }

//     return ErrorStatusLocVar;
//  }

// // static u8* USART1_BufferValue;
// // static u8* USART2_BufferValue;
// // static u8* USART6_BufferValue;
// // USART_ErrorStatus_t USART_SendBufferZeroCopy(USART_Channels_t Channel, u8* buffer, u32 length){
// //     USART_ErrorStatus_t ErrorStatusLocVar = USART_Ok;
// //     if(buffer == NULL ){
// //         ErrorStatusLocVar = USART_NULLPOINTER;
// //     }
// //     else if(TXReq[Channel].state == USART_Busy){
// //         ErrorStatusLocVar = USART_BUSY;
// //     }
// //     else{
// //         switch (Channel)
// //         {
// //         case USART1_channel:
// //             /* make TXEIE --> 1*/
// //             USART1->USART_CR1 |= USART_MASK_TXEIE_ENABLE;
// //             if(TXReq[USART1_channel].state == USART_Ready){
// //                 TXReq[USART1_channel].state = USART_Busy;
// //             }
// //             else{
// //                 ErrorStatusLocVar = USART_BUSY;
// //             }
// //             TXReq[USART1_channel].buffer.data = buffer;
// //             TXReq[USART1_channel].buffer.size = length;
// //             TXReq[USART1_channel].buffer.pos = 0;
// //             USART1->USART_DR = TXReq[USART1_channel].buffer.data[0];
// //             TXReq[USART1_channel].buffer.pos++;
// //         break;
// //         }
// // }

// void USART1_IRQHandler(void){
//     if((USART1->USART_SR >> TXE) & 0x01){//(USART1->USART_SR >> TC) & 0x01
//         if(TXReq[USART1_channel].buffer.pos < TXReq[USART1_channel].buffer.size){
//             USART1->USART_SR &= USART_MASK_CLEAR_TC;
//             u32 index = TXReq[USART1_channel].buffer.pos;
//             USART1->USART_DR = TXReq[USART1_channel].buffer.data[index];
//             TXReq[USART1_channel].buffer.pos++;
//         }
//         else{
//             TXReq[USART1_channel].state = USART_Ready;
//             if(TXReq[USART1_channel].cb){
//                 TXReq[USART1_channel].cb();
//             }
//             USART1->USART_SR &= USART_MASK_CLEAR_TC;
//         }
//        // USART1_Finish_sending = USART_FINISH;
//     }
//     if((USART1->USART_SR >> RXNE) & 0x01){
//         if(RXReq[USART1_channel].buffer.pos < RXReq[USART1_channel].buffer.size){
//             u32 index = RXReq[USART1_channel].buffer.pos;
//             RXReq[USART1_channel].buffer.data[index] = USART1->USART_DR;
//             RXReq[USART1_channel].buffer.pos++;
//         }
//         else if(RXReq[USART1_channel].buffer.pos == RXReq[USART1_channel].buffer.size){
//             RXReq[USART1_channel].state = USART_Ready;
//             RXReq[USART1_channel].buffer.pos = 0;
//             RXReq[USART1_channel].buffer.size = 0;
//             if(RXReq[USART1_channel].cb){
//                 RXReq[USART1_channel].cb();
//             }
//            // USART1_Finish_Receiving = USART_FINISH;
//         }
//     }
// }

// void USART2_IRQHandler(void){
// 	if((USART2->USART_SR >> TXE) & 0x01){//(USART2->USART_SR >> TC) & 0x01
// 		if(TXReq[USART2_channel].buffer.pos < TXReq[USART2_channel].buffer.size){
// 			u32 index = TXReq[USART2_channel].buffer.pos;
// 			USART2->USART_DR = TXReq[USART2_channel].buffer.data[index];
// 			TXReq[USART2_channel].buffer.pos++;
// 		}
// 		else{
// 			TXReq[USART2_channel].state = USART_Ready;
// 			if(TXReq[USART2_channel].cb){
// 				TXReq[USART2_channel].cb();
// 			}
// 		}
// 	}
// 	if((USART2->USART_SR >> RXNE) & 0x01){
// 		if(TXReq[USART2_channel].buffer.pos < TXReq[USART2_channel].buffer.size){
// 			u32 index = TXReq[USART2_channel].buffer.pos;
// 			TXReq[USART2_channel].buffer.data[index] = USART2->USART_DR;
// 			TXReq[USART2_channel].buffer.pos++;
// 		}
// 		else if(TXReq[USART2_channel].buffer.pos == TXReq[USART2_channel].buffer.size){
// 			TXReq[USART2_channel].state = USART_Ready;
// 			TXReq[USART2_channel].buffer.pos = 0;
// 			TXReq[USART2_channel].buffer.size = 0;
// 			//USART2_Finish_Receiving = USART_FINISH;
// 		}
// 	}
// }

// void USART6_IRQHandler(void){
// 	if((USART6->USART_SR >> TXE) & 0x01){//(USART6->USART_SR >> TC) & 0x01
// 		if(TXReq[USART6_channel].buffer.pos < TXReq[USART6_channel].buffer.size){
// 			u32 index = TXReq[USART6_channel].buffer.pos;
// 			USART6->USART_DR = TXReq[USART6_channel].buffer.data[index];
// 			TXReq[USART6_channel].buffer.pos++;
// 		}
// 		else{
// 			TXReq[USART6_channel].state = USART_Ready;
// 			if(TXReq[USART6_channel].cb){
// 				TXReq[USART6_channel].cb();
// 			}
// 		}
// 	}
// 	if((USART6->USART_SR >> RXNE) & 0x01){
// 		if(TXReq[USART6_channel].buffer.pos < TXReq[USART6_channel].buffer.size){
// 			u32 index = TXReq[USART6_channel].buffer.pos;
// 			TXReq[USART6_channel].buffer.data[index] = USART6->USART_DR;
// 			TXReq[USART6_channel].buffer.pos++;
// 		}
// 		else if(TXReq[USART6_channel].buffer.pos == TXReq[USART6_channel].buffer.size){
// 			TXReq[USART6_channel].state = USART_Ready;
// 			TXReq[USART6_channel].buffer.pos = 0;
// 			TXReq[USART6_channel].buffer.size = 0;
// 			//USART6_Finish_Receiving = USART_FINISH;
// 		}
// 	}
// }
