/*
 * NVIC.h
 *
 *  Created on: Mar 4, 2024
 *      Author: Dell
 */

#ifndef NVIC_H_
#define NVIC_H_

#include "std_types.h"

#define Group_priority_4Bits 0x05FA0300    /*Group Priority Bits:4 [7:4] &  SubPriority Bits:0  [None]*/
#define Group_priority_3Bits 0x05FA0400    /*Group Priority Bits:3 [7:5] &  SubPriority Bits:1  [4]*/
#define Group_priority_2Bits 0x05FA0500    /*Group Priority Bits:2 [7:6] &  SubPriority Bits:2  [5:4]*/
#define Group_priority_1Bit  0x05FA0600    /*Group Priority Bits:1 [7]   &  SubPriority Bits:3  [6:4]*/
#define Group_priority_None  0x05FA0700    /*Group Priority Bits:0 [None] & SubPriority Bits:4  [7:4]*/

typedef enum{
     NVIC_OK,
	 NVIC_Invalid_IRQn,
	 NVIC_Invalid_PriorityGrouping,
	 NVIC_InvalidGroupPriority_Value,
	 NVIC_InvalidSubPriority_Value,
	 NVIC_ErrorNullPTR
}NVIC_ErrorStatus_t;

/* functions prototypes */

/*************************** NVIC_EnableInterrupt ***********************************************
 * @brief:  This function enables an interrupt. ------------------------------------------------*
 * @param_in:  IRQn (Interrupt number) : Macro begins with NVIC_ listed in stm32f401cc.h -------*
 * @return: NVIC_ErrorStatus_t, can be:---------------------------------------------------------*
 *         - NVIC_Invalid_IRQn: the user entered IRQn out of range (>85 in our case) -----------*
 ***********************************************************************************************/
NVIC_ErrorStatus_t NVIC_EnableInterrupt(u8 IRQn);

/*************************** NVIC_DisableInterrupt ***********************************************
 * @brief:  This function disables an interrupt. ------------------------------------------------*
 * @param_in:  IRQn (Interrupt number) : Macro begins with NVIC_ listed in stm32f401cc.h --------*
 * @return: NVIC_ErrorStatus_t, can be:----------------------------------------------------------*
 *         - NVIC_Invalid_IRQn: the user entered IRQn out of range (>85 in our case) ------------*
 ************************************************************************************************/
NVIC_ErrorStatus_t NVIC_DisableInterrupt(u8 IRQn);

/*************************** NVIC_SetPending *****************************************************
 * @brief:  This function set the pending of an interrupt. --------------------------------------*
 * @param_in:  IRQn (Interrupt number) : Macro begins with NVIC_ listed in stm32f401cc.h --------*
 * @return: NVIC_ErrorStatus_t, can be:----------------------------------------------------------*
 *         - NVIC_Invalid_IRQn: the user entered IRQn out of range (>85 in our case) ------------*
 ************************************************************************************************/
NVIC_ErrorStatus_t NVIC_SetPending(u8 IRQn);

/*************************** NVIC_ClearPending ****************************************************
 * @brief:  This function clear the pending of an interrupt. -------------------------------------*
 * @param_in:  IRQn (Interrupt number) : Macro begins with NVIC_ listed in stm32f401cc.h ---------*
 * @return: NVIC_ErrorStatus_t, can be:-----------------------------------------------------------*
 *          * NVIC_Invalid_IRQn: the user entered IRQn out of range (>85 in our case) ------------*
 *************************************************************************************************/
NVIC_ErrorStatus_t NVIC_ClearPending(u8 IRQn);

/*************************** NVIC_SetPriority ******************************************************
 * @brief:  This function set the priority bits of an interrupt. ----------------------------------*
 * @param_in:  IRQn (Interrupt number) : Macro begins with NVIC_ listed in stm32f401cc.h ------- --*
 * @param_in:  PriorityGrouping: This field determines the split of group priority from subpriority*
 *          Options: Group_priority_4Bits  --------------------------------------------------------*
 *                   Group_priority_3Bits  --------------------------------------------------------*
 *                   Group_priority_2Bits  --------------------------------------------------------*
 *                   Group_priority_1Bit  ---------------------------------------------------------*
 *                   Group_priority_None ----------------------------------------------------------*
 * @param_in:  GroupPriority: Group priority value. -----------------------------------------------*
 *          Options: Choose according to the selected PriorityGrouping.----------------------------*
 *                   number must less than 2^(GroupPriority bits) ---------------------------------*
 * @param_in:  SubPriority: Subpriority value. ----------------------------------------------------*
 *          Options: Choose according to the selected PriorityGrouping.----------------------------*
 *                   number must less than 2^(Subpriority bits) -----------------------------------*
 * @return: NVIC_ErrorStatus_t: possible return Error status  -------------------------------------*
 *          - NVIC_OK: Operation successful -------------------------------------------------------*
 *          - NVIC_Invalid_IRQn: Invalid interrupt number -----------------------------------------*
 *          - NVIC_Invalid_PriorityGrouping: Invalid priority grouping value ----------------------*
 *          - NVIC_InvalidGroupPriority_Value: Invalid group priority value -----------------------*
 *          - NVIC_InvalidSubPriority_Value: Invalid subpriority value ----------------------------*
 **************************************************************************************************/
NVIC_ErrorStatus_t NVIC_SetPriority(u8 IRQn,u32 PriorityGrouping,u8 GroupPriorty,u8 SubPriority);

/*************************** NVIC_GetPriority *******************************************************
 * @brief:  Get the priority for an interrupt.------------------------------------------------------*
 * @param_in:  IRQn (Interrupt number): Macro begins with NVIC_ in stm32f401cc.h.-------------------*
 * @param_out:  Priority: Pointer to store the priority value.--------------------------------------*
 * @return: NVIC_ErrorStatus_t, possible values:----------------------------------------------------*
 *          - NVIC_OK: Operation successful.--------------------------------------------------------*
 *          - NVIC_Invalid_IRQn: Invalid interrupt number.------------------------------------------*
 *          - NVIC_ErrorNullPTR: Null pointer error ------------------------------------------------*
 ***************************************************************************************************/
NVIC_ErrorStatus_t NVIC_GetPriority(u8 IRQn,u8* Priority);

/*************************** NVIC_GetActiveStatus ***************************************************
 * @brief:  Get the priority for an interrupt.------------------------------------------------------*
 * @param_in:  IRQn (Interrupt number): Macro begins with NVIC_ in stm32f401cc.h.-------------------*
 * @param_out:  ActiveStatus: Pointer to store the Active Status of the interrupt.------------------*
 * @return: NVIC_ErrorStatus_t, possible values:----------------------------------------------------*
 *          - NVIC_OK: Operation successful.--------------------------------------------------------*
 *          - NVIC_Invalid_IRQn: Invalid interrupt number.------------------------------------------*
 *          - NVIC_ErrorNullPTR: Null pointer error ------------------------------------------------*
 ***************************************************************************************************/
NVIC_ErrorStatus_t NVIC_GetActiveStatus(u8 IRQn, u8* ActiveStatus);

/*************************** NVIC_GenerateSWI ********************************************************
 * @brief:  This function generates a software interrupt. -------------------------------------------*
 * @param_in:  IRQn (Interrupt number) : Macro begins with NVIC_ listed in stm32f401cc.h ------------*
 * @return: NVIC_ErrorStatus_t, can be:--------------------------------------------------------------*
 *         - NVIC_Invalid_IRQn: the user entered IRQn out of range (>85 in our case) ----------------*
 ****************************************************************************************************/
NVIC_ErrorStatus_t NVIC_GenerateSWI(u8 IRQn);

#endif /* NVIC_H_ */
