/*
 * NVIC.c
 *
 *  Created on: Mar 4, 2024
 *      Author: Dell
 */
#include "NVIC.h"

/* the max number of IRQn */
#define IRQn_MAX   85
/* Mask for priority grouping*/
#define PriorityGrouping_Mask 0xffff0700
/* mask for IPR register */
#define IPR_Mask 0xf0
/* No of Bits that the processor implemented in NVIC_IPR*/
#define ImplementedPriorityBits 4
/*NVIC Base Address*/
#define NVIC_BASEADDRESS (0xE000E100UL)
/* the base address as pointer to the NVIC registers*/
#define NVIC ((void* )NVIC_BASEADDRESS)
/*SCB Base Address*/
#define SCB_BASEADDRESS (0xE000E008UL)
/* the base address as pointer to the SCB registers*/
#define SCB ((void* )SCB_BASEADDRESS)

/* NVIC Registers definitions */
typedef struct
{
	volatile u32 ISER[8];
    volatile u32 Reserved1[24];
    volatile u32 ICER[8];
    volatile u32 Reserved2[24];
    volatile u32 ISPR[8];
    volatile u32 Reserved3[24];
    volatile u32 ICPR[8];
    volatile u32 Reserved4[24];
    volatile u32 IABR[8];
    volatile u32 Reserved5[56];
    volatile u8  IPR[240]; // volatile u32 IPR[60];
    volatile u32 Reserved6[644];
    volatile u32 STIR;
}NVIC_RegisterOffset_t;

/* SCB Registers definitions */
typedef struct
{
	volatile u32 ACTLR;
	volatile u32 Reserved1[829];
	volatile u32 CPUID;
	volatile u32 ICSR;
	volatile u32 VTOR;
	volatile u32 AIRCR;
	volatile u32 SCR;
	volatile u32 CCR;
	volatile u32 SHPR1;
	volatile u32 SHPR2;
	volatile u32 SHPR3;
	volatile u32 SHCSR;
	volatile u32 CFSR;
	volatile u32 HFSR;
	volatile u32 Reserved;
	volatile u32 MMAR;
	volatile u32 BFAR;
	volatile u32 AFSR;
}SCB_RegisterOffset_t;


u8 power(u8 base,u8 exponent)
{
	u8 result=1;
	    for (u8 i=0;i<exponent;i++)
	    {
	        result*=base;
	    }
    return result;
}
NVIC_ErrorStatus_t NVIC_EnableInterrupt(u8 IRQn)
{
	NVIC_ErrorStatus_t NVIC_RetErrorState=NVIC_OK;
	if(IRQn>=IRQn_MAX)
	{
		NVIC_RetErrorState=NVIC_Invalid_IRQn;
	}
	((NVIC_RegisterOffset_t*)(NVIC))->ISER[IRQn/32]=(1UL<<(IRQn%32));

	return NVIC_RetErrorState;
}
NVIC_ErrorStatus_t NVIC_DisableInterrupt(u8 IRQn)
{
	NVIC_ErrorStatus_t NVIC_RetErrorState=NVIC_OK;
		if(IRQn>=IRQn_MAX)
		{
			NVIC_RetErrorState=NVIC_Invalid_IRQn;
		}
	((NVIC_RegisterOffset_t*)(NVIC))->ICER[IRQn/32]=(1UL<<(IRQn%32));

		return NVIC_RetErrorState;
}
NVIC_ErrorStatus_t NVIC_SetPending(u8 IRQn)
{
	NVIC_ErrorStatus_t NVIC_RetErrorState=NVIC_OK;
		if(IRQn>=IRQn_MAX)
		{
			NVIC_RetErrorState=NVIC_Invalid_IRQn;
		}
	((NVIC_RegisterOffset_t*)(NVIC))->ISPR[IRQn/32]=(1UL<<(IRQn%32));

		return NVIC_RetErrorState;
}
NVIC_ErrorStatus_t NVIC_ClearPending(u8 IRQn)
{
	NVIC_ErrorStatus_t NVIC_RetErrorState=NVIC_OK;
		if(IRQn>=IRQn_MAX)
		{
			NVIC_RetErrorState=NVIC_Invalid_IRQn;
		}
	((NVIC_RegisterOffset_t*)(NVIC))->ICPR[IRQn/32]=(1UL<<(IRQn%32));

		return NVIC_RetErrorState;
}
NVIC_ErrorStatus_t NVIC_GetActiveStatus(u8 IRQn, u8* ActiveStatus)
{
	u8 Reg_num;
	u8 Bit_num;
	NVIC_ErrorStatus_t NVIC_RetErrorState=NVIC_OK;
			if(IRQn>=IRQn_MAX)
			{
				NVIC_RetErrorState=NVIC_Invalid_IRQn;
			}
			else if(ActiveStatus==NULLPTR)
			{
				NVIC_RetErrorState=NVIC_ErrorNullPTR;
			}
			else
			{
				Reg_num=IRQn/32;
				Bit_num=IRQn%32;
				*ActiveStatus=(((NVIC_RegisterOffset_t*)(NVIC))->IABR[Reg_num]>>Bit_num)&1UL;
			}
			return NVIC_RetErrorState;
}

NVIC_ErrorStatus_t NVIC_SetPriority(u8 IRQn,u32 PriorityGrouping,u8 GroupPriorty,u8 SubPriority)
{
	u32 Loc_RegisterValue;
	/* extract PRIGROUP bits value*/
	u8 PRIGROUP_Value=(PriorityGrouping >> 8) & 0x07;
	u8 SubPriorityBits_Num=PRIGROUP_Value-3; //calculate the number of subpriority bits
	u8 GroupPriorityBits_Num=ImplementedPriorityBits-SubPriorityBits_Num; //calculate the number of grouppriority bits
	u8 Max_GroupPriority_Value=power(2,GroupPriorityBits_Num)-1;
	u8 Max_SubPriority_Value=power(2,SubPriorityBits_Num)-1;

	NVIC_ErrorStatus_t NVIC_RetErrorState=NVIC_OK;
	if((PriorityGrouping<Group_priority_4Bits)||(PriorityGrouping>Group_priority_None))
	{
		NVIC_RetErrorState= NVIC_Invalid_PriorityGrouping;
	}
	else if(GroupPriorty>Max_GroupPriority_Value)
	{
		NVIC_RetErrorState= NVIC_InvalidGroupPriority_Value;
	}
	else if(GroupPriorty>Max_SubPriority_Value)
	{
		NVIC_RetErrorState= NVIC_InvalidSubPriority_Value;
	}

	else
	{
		/* Set Priority Grouping Field in SCB_AIRCR Register */
		Loc_RegisterValue=((SCB_RegisterOffset_t*)(SCB))->AIRCR;
		Loc_RegisterValue&=~(PriorityGrouping_Mask);
		Loc_RegisterValue|=PriorityGrouping;
		((SCB_RegisterOffset_t*)(SCB))->AIRCR=Loc_RegisterValue;

		/* Set Priority Value in NVIC_IPR[IRQn] Register */
		Loc_RegisterValue=((NVIC_RegisterOffset_t*)(NVIC))->IPR[IRQn];
		Loc_RegisterValue&=~(IPR_Mask);
		Loc_RegisterValue|=(SubPriority<<ImplementedPriorityBits)|(GroupPriorty<<(ImplementedPriorityBits+SubPriorityBits_Num));
		((NVIC_RegisterOffset_t*)(NVIC))->IPR[IRQn]|=Loc_RegisterValue;
	}
	return NVIC_RetErrorState;
}

NVIC_ErrorStatus_t NVIC_GetPriority(u8 IRQn,u8* Priority)
{
    NVIC_ErrorStatus_t NVIC_RetErrorState=NVIC_OK;
	if(IRQn>=IRQn_MAX)
	{
		NVIC_RetErrorState=NVIC_Invalid_IRQn;
	}
	else if (Priority==NULLPTR)
	{
		NVIC_RetErrorState=NVIC_ErrorNullPTR;
	}
	else
	{
		*Priority=(((NVIC_RegisterOffset_t*)(NVIC))->IPR[IRQn]&IPR_Mask)>>ImplementedPriorityBits;
	}
	return NVIC_RetErrorState;
}
NVIC_ErrorStatus_t NVIC_GenerateSWI(u8 IRQn)
{
    NVIC_ErrorStatus_t NVIC_RetErrorState=NVIC_OK;
	if(IRQn>=IRQn_MAX)
	{
		NVIC_RetErrorState=NVIC_Invalid_IRQn;
	}
	((NVIC_RegisterOffset_t*)(NVIC))->STIR=IRQn;

	return NVIC_RetErrorState;
}
