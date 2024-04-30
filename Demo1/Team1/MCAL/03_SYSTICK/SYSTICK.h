/*
 * SYSTICK.h
 *
 * Created: 3/11/2024 5:53:43 PM
 *  Author: Eman
 */ 


#ifndef SYSTICK_H_
#define SYSTICK_H_

/************************************************************************************
 *                                       datatypes                                  *
 * **********************************************************************************/
// typedef unsigned char         u8;
// typedef signed char           s8;
// typedef short unsigned int    u16;
// typedef short signed int      s16;
// typedef unsigned int          u32;
// typedef signed int            s32;
#include "STD_LIB/std_types.h"
typedef enum
{
	STK_Ok,
	STK_WrongSOURCECLK,
	STK_NULLPOINTER,
	STK_WRONGTIMEVALUE
}STK_ErrorStatus_t;

typedef struct{
	volatile u32 STK_CTRL;
	volatile u32 STK_LOAD;
	volatile u32 STK_VAL;
	volatile u32 STK_CALIB;
}STK_Register_t;

/* for callback function */
typedef void (*systickcbf_t)(void);
/************************************************************************************
 *                                       #defines                                  *
 * **********************************************************************************/

#define SOURCE_AHB                       0x00000004
#define SOURCE_AHB_DIV8                  0xFFFFFFFB
/************************************************************************************
 *                                       functions                                  *
 * **********************************************************************************/

STK_ErrorStatus_t STK_Init(u32 Copy_ClkSource);
STK_ErrorStatus_t STK_SetTime_mS(u32 Copy_TimeValue);
void STK_EnableSTKInterrupt(void);
void STK_DisableSTKInterrupt(void);
void STK_Start(void);
void STK_Stop(void);
STK_ErrorStatus_t STK_SetCallback(systickcbf_t Callback);
#endif /* SYSTICK_H_ */