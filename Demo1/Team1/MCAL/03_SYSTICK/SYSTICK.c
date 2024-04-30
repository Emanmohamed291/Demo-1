/*
 * SYSTICK.c
 *
 * Created: 3/11/2024 5:53:58 PM
 *  Author: Eman
 */ 
#include "SYSTICK.h"

#define STK                       ((STK_Register_t*)0xE000E010) 
#define NULL                      ((void *)0)
#define STK_ENABLE_TICKINT        0x00000002
#define STK_DISABLE_TICKINT       0xFFFFFFFD
#define STK_ENABLE_COUNT          0x00000001
#define STK_DISABLE_COUNT         0xFFFFFFFE
#define CLK_AHB_FREQ              16000000
#define MAX_TIME_VALUE            16777215


u32 Clk_Source;
static systickcbf_t Systick_Callback = NULL;


/************************************************************************************
 *                                       functions                                  *
 * **********************************************************************************/

STK_ErrorStatus_t STK_Init(u32 Copy_ClkSource){
    STK_ErrorStatus_t ErrorStatusLocVar = STK_Ok;
    if(Copy_ClkSource != SOURCE_AHB &&Copy_ClkSource != SOURCE_AHB_DIV8){
        ErrorStatusLocVar = STK_WrongSOURCECLK;
    }
    else if(Copy_ClkSource == SOURCE_AHB){
        STK->STK_CTRL |= Copy_ClkSource;
        Clk_Source = Copy_ClkSource;
    }
    else{
        STK->STK_CTRL &= Copy_ClkSource;
        Clk_Source = Copy_ClkSource;
    }
    return ErrorStatusLocVar;
}

void STK_EnableSTKInterrupt(void){
    STK->STK_CTRL |= STK_ENABLE_TICKINT;
}

void STK_DisableSTKInterrupt(void){
    STK->STK_CTRL &= STK_DISABLE_TICKINT; 
}
/*
*/

STK_ErrorStatus_t STK_SetTime_mS(u32 Copy_TimeValue){
    u32 Loc_time = 0;
    STK_ErrorStatus_t ErrorStatusLocVar = STK_Ok;
    if(Clk_Source == SOURCE_AHB_DIV8){
        Loc_time = (CLK_AHB_FREQ/8000)*Copy_TimeValue -1;
    }
    else if(Clk_Source == SOURCE_AHB){
        Loc_time = (CLK_AHB_FREQ/1000)*Copy_TimeValue -1;
    }
    if(Loc_time > MAX_TIME_VALUE){
        ErrorStatusLocVar = STK_WRONGTIMEVALUE;
    }
    else{
        STK->STK_LOAD = Loc_time;
    }
    return ErrorStatusLocVar;
}
void STK_Start(void){
    STK->STK_CTRL |= STK_ENABLE_COUNT;
}
void STK_Stop(void){
    STK->STK_CTRL &=STK_DISABLE_COUNT;
}
STK_ErrorStatus_t STK_SetCallback(systickcbf_t Copy_pfCallback){
    STK_ErrorStatus_t ErrorStatusLocVar = STK_Ok;
    if(Copy_pfCallback == NULL){
        ErrorStatusLocVar = STK_NULLPOINTER;
    }
    else{
        Systick_Callback = Copy_pfCallback;
    }
    return ErrorStatusLocVar;
}

void SysTick_Handler(void)
{
    if (Systick_Callback){
	    Systick_Callback();
	}
	else{
        /* DO NOTHING */
	}
}