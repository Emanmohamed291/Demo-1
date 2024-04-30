/* AUTOGENERATED FILE. DO NOT EDIT. */
#ifndef _MOCK_GPIO_H
#define _MOCK_GPIO_H

#include "unity.h"
#include "GPIO.h"

/* Ignore the following warnings, since we are copying code */
#if defined(__GNUC__) && !defined(__ICC) && !defined(__TMS470__)
#if __GNUC__ > 4 || (__GNUC__ == 4 && (__GNUC_MINOR__ > 6 || (__GNUC_MINOR__ == 6 && __GNUC_PATCHLEVEL__ > 0)))
#pragma GCC diagnostic push
#endif
#if !defined(__clang__)
#pragma GCC diagnostic ignored "-Wpragmas"
#endif
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#pragma GCC diagnostic ignored "-Wduplicate-decl-specifier"
#endif

void mock_GPIO_Init(void);
void mock_GPIO_Destroy(void);
void mock_GPIO_Verify(void);




#define GPIO_InitPin_IgnoreAndReturn(cmock_retval) GPIO_InitPin_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void GPIO_InitPin_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, ErrorStatus_t cmock_to_return);
#define GPIO_InitPin_StopIgnore() GPIO_InitPin_CMockStopIgnore()
void GPIO_InitPin_CMockStopIgnore(void);
#define GPIO_InitPin_ExpectAndReturn(GPIO_Config, cmock_retval) GPIO_InitPin_CMockExpectAndReturn(__LINE__, GPIO_Config, cmock_retval)
void GPIO_InitPin_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, GPIO_CFG_t* GPIO_Config, ErrorStatus_t cmock_to_return);
typedef ErrorStatus_t (* CMOCK_GPIO_InitPin_CALLBACK)(GPIO_CFG_t* GPIO_Config, int cmock_num_calls);
void GPIO_InitPin_AddCallback(CMOCK_GPIO_InitPin_CALLBACK Callback);
void GPIO_InitPin_Stub(CMOCK_GPIO_InitPin_CALLBACK Callback);
#define GPIO_InitPin_StubWithCallback GPIO_InitPin_Stub
#define GPIO_SetPinValue_IgnoreAndReturn(cmock_retval) GPIO_SetPinValue_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void GPIO_SetPinValue_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, ErrorStatus_t cmock_to_return);
#define GPIO_SetPinValue_StopIgnore() GPIO_SetPinValue_CMockStopIgnore()
void GPIO_SetPinValue_CMockStopIgnore(void);
#define GPIO_SetPinValue_ExpectAndReturn(GPIO_Port, GPIO_Pin, GPIO_PinState, cmock_retval) GPIO_SetPinValue_CMockExpectAndReturn(__LINE__, GPIO_Port, GPIO_Pin, GPIO_PinState, cmock_retval)
void GPIO_SetPinValue_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, void* GPIO_Port, uint32_t GPIO_Pin, uint32_t GPIO_PinState, ErrorStatus_t cmock_to_return);
typedef ErrorStatus_t (* CMOCK_GPIO_SetPinValue_CALLBACK)(void* GPIO_Port, uint32_t GPIO_Pin, uint32_t GPIO_PinState, int cmock_num_calls);
void GPIO_SetPinValue_AddCallback(CMOCK_GPIO_SetPinValue_CALLBACK Callback);
void GPIO_SetPinValue_Stub(CMOCK_GPIO_SetPinValue_CALLBACK Callback);
#define GPIO_SetPinValue_StubWithCallback GPIO_SetPinValue_Stub
#define GPIO_GetPinValue_IgnoreAndReturn(cmock_retval) GPIO_GetPinValue_CMockIgnoreAndReturn(__LINE__, cmock_retval)
void GPIO_GetPinValue_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, ErrorStatus_t cmock_to_return);
#define GPIO_GetPinValue_StopIgnore() GPIO_GetPinValue_CMockStopIgnore()
void GPIO_GetPinValue_CMockStopIgnore(void);
#define GPIO_GetPinValue_ExpectAndReturn(GPIO_Port, GPIO_Pin, GPIO_PinState, cmock_retval) GPIO_GetPinValue_CMockExpectAndReturn(__LINE__, GPIO_Port, GPIO_Pin, GPIO_PinState, cmock_retval)
void GPIO_GetPinValue_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, void* GPIO_Port, uint32_t GPIO_Pin, uint32_t* GPIO_PinState, ErrorStatus_t cmock_to_return);
typedef ErrorStatus_t (* CMOCK_GPIO_GetPinValue_CALLBACK)(void* GPIO_Port, uint32_t GPIO_Pin, uint32_t* GPIO_PinState, int cmock_num_calls);
void GPIO_GetPinValue_AddCallback(CMOCK_GPIO_GetPinValue_CALLBACK Callback);
void GPIO_GetPinValue_Stub(CMOCK_GPIO_GetPinValue_CALLBACK Callback);
#define GPIO_GetPinValue_StubWithCallback GPIO_GetPinValue_Stub

#if defined(__GNUC__) && !defined(__ICC) && !defined(__TMS470__)
#if __GNUC__ > 4 || (__GNUC__ == 4 && (__GNUC_MINOR__ > 6 || (__GNUC_MINOR__ == 6 && __GNUC_PATCHLEVEL__ > 0)))
#pragma GCC diagnostic pop
#endif
#endif

#endif