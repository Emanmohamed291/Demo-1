/* AUTOGENERATED FILE. DO NOT EDIT. */
#include <string.h>
#include <stdlib.h>
#include <setjmp.h>
#include "cmock.h"
#include "mock_GPIO.h"

static const char* CMockString_GPIO_Config = "GPIO_Config";
static const char* CMockString_GPIO_GetPinValue = "GPIO_GetPinValue";
static const char* CMockString_GPIO_InitPin = "GPIO_InitPin";
static const char* CMockString_GPIO_Pin = "GPIO_Pin";
static const char* CMockString_GPIO_PinState = "GPIO_PinState";
static const char* CMockString_GPIO_Port = "GPIO_Port";
static const char* CMockString_GPIO_SetPinValue = "GPIO_SetPinValue";

typedef struct _CMOCK_GPIO_InitPin_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  ErrorStatus_t ReturnVal;
  int CallOrder;
  GPIO_CFG_t* Expected_GPIO_Config;

} CMOCK_GPIO_InitPin_CALL_INSTANCE;

typedef struct _CMOCK_GPIO_SetPinValue_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  ErrorStatus_t ReturnVal;
  int CallOrder;
  void* Expected_GPIO_Port;
  uint32_t Expected_GPIO_Pin;
  uint32_t Expected_GPIO_PinState;

} CMOCK_GPIO_SetPinValue_CALL_INSTANCE;

typedef struct _CMOCK_GPIO_GetPinValue_CALL_INSTANCE
{
  UNITY_LINE_TYPE LineNumber;
  ErrorStatus_t ReturnVal;
  int CallOrder;
  void* Expected_GPIO_Port;
  uint32_t Expected_GPIO_Pin;
  uint32_t* Expected_GPIO_PinState;

} CMOCK_GPIO_GetPinValue_CALL_INSTANCE;

static struct mock_GPIOInstance
{
  char GPIO_InitPin_IgnoreBool;
  ErrorStatus_t GPIO_InitPin_FinalReturn;
  char GPIO_InitPin_CallbackBool;
  CMOCK_GPIO_InitPin_CALLBACK GPIO_InitPin_CallbackFunctionPointer;
  int GPIO_InitPin_CallbackCalls;
  CMOCK_MEM_INDEX_TYPE GPIO_InitPin_CallInstance;
  char GPIO_SetPinValue_IgnoreBool;
  ErrorStatus_t GPIO_SetPinValue_FinalReturn;
  char GPIO_SetPinValue_CallbackBool;
  CMOCK_GPIO_SetPinValue_CALLBACK GPIO_SetPinValue_CallbackFunctionPointer;
  int GPIO_SetPinValue_CallbackCalls;
  CMOCK_MEM_INDEX_TYPE GPIO_SetPinValue_CallInstance;
  char GPIO_GetPinValue_IgnoreBool;
  ErrorStatus_t GPIO_GetPinValue_FinalReturn;
  char GPIO_GetPinValue_CallbackBool;
  CMOCK_GPIO_GetPinValue_CALLBACK GPIO_GetPinValue_CallbackFunctionPointer;
  int GPIO_GetPinValue_CallbackCalls;
  CMOCK_MEM_INDEX_TYPE GPIO_GetPinValue_CallInstance;
} Mock;

extern jmp_buf AbortFrame;
extern int GlobalExpectCount;
extern int GlobalVerifyOrder;

void mock_GPIO_Verify(void)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_MEM_INDEX_TYPE call_instance;
  call_instance = Mock.GPIO_InitPin_CallInstance;
  if (Mock.GPIO_InitPin_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_GPIO_InitPin);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  if (Mock.GPIO_InitPin_CallbackFunctionPointer != NULL)
  {
    call_instance = CMOCK_GUTS_NONE;
    (void)call_instance;
  }
  call_instance = Mock.GPIO_SetPinValue_CallInstance;
  if (Mock.GPIO_SetPinValue_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_GPIO_SetPinValue);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  if (Mock.GPIO_SetPinValue_CallbackFunctionPointer != NULL)
  {
    call_instance = CMOCK_GUTS_NONE;
    (void)call_instance;
  }
  call_instance = Mock.GPIO_GetPinValue_CallInstance;
  if (Mock.GPIO_GetPinValue_IgnoreBool)
    call_instance = CMOCK_GUTS_NONE;
  if (CMOCK_GUTS_NONE != call_instance)
  {
    UNITY_SET_DETAIL(CMockString_GPIO_GetPinValue);
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLess);
  }
  if (Mock.GPIO_GetPinValue_CallbackFunctionPointer != NULL)
  {
    call_instance = CMOCK_GUTS_NONE;
    (void)call_instance;
  }
}

void mock_GPIO_Init(void)
{
  mock_GPIO_Destroy();
}

void mock_GPIO_Destroy(void)
{
  CMock_Guts_MemFreeAll();
  memset(&Mock, 0, sizeof(Mock));
  GlobalExpectCount = 0;
  GlobalVerifyOrder = 0;
}

ErrorStatus_t GPIO_InitPin(GPIO_CFG_t* GPIO_Config)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_GPIO_InitPin_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_GPIO_InitPin);
  cmock_call_instance = (CMOCK_GPIO_InitPin_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.GPIO_InitPin_CallInstance);
  Mock.GPIO_InitPin_CallInstance = CMock_Guts_MemNext(Mock.GPIO_InitPin_CallInstance);
  if (Mock.GPIO_InitPin_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    if (cmock_call_instance == NULL)
      return Mock.GPIO_InitPin_FinalReturn;
    memcpy((void*)(&Mock.GPIO_InitPin_FinalReturn), (void*)(&cmock_call_instance->ReturnVal),
         sizeof(ErrorStatus_t[sizeof(cmock_call_instance->ReturnVal) == sizeof(ErrorStatus_t) ? 1 : -1])); /* add ErrorStatus_t to :treat_as_array if this causes an error */
    return cmock_call_instance->ReturnVal;
  }
  if (!Mock.GPIO_InitPin_CallbackBool &&
      Mock.GPIO_InitPin_CallbackFunctionPointer != NULL)
  {
    ErrorStatus_t cmock_cb_ret = Mock.GPIO_InitPin_CallbackFunctionPointer(GPIO_Config, Mock.GPIO_InitPin_CallbackCalls++);
    UNITY_CLR_DETAILS();
    return cmock_cb_ret;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_GPIO_InitPin,CMockString_GPIO_Config);
    UNITY_TEST_ASSERT_EQUAL_MEMORY((void*)(cmock_call_instance->Expected_GPIO_Config), (void*)(GPIO_Config), sizeof(GPIO_CFG_t), cmock_line, CMockStringMismatch);
  }
  if (Mock.GPIO_InitPin_CallbackFunctionPointer != NULL)
  {
    cmock_call_instance->ReturnVal = Mock.GPIO_InitPin_CallbackFunctionPointer(GPIO_Config, Mock.GPIO_InitPin_CallbackCalls++);
  }
  UNITY_CLR_DETAILS();
  return cmock_call_instance->ReturnVal;
}

void CMockExpectParameters_GPIO_InitPin(CMOCK_GPIO_InitPin_CALL_INSTANCE* cmock_call_instance, GPIO_CFG_t* GPIO_Config);
void CMockExpectParameters_GPIO_InitPin(CMOCK_GPIO_InitPin_CALL_INSTANCE* cmock_call_instance, GPIO_CFG_t* GPIO_Config)
{
  cmock_call_instance->Expected_GPIO_Config = GPIO_Config;
}

void GPIO_InitPin_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, ErrorStatus_t cmock_to_return)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_GPIO_InitPin_CALL_INSTANCE));
  CMOCK_GPIO_InitPin_CALL_INSTANCE* cmock_call_instance = (CMOCK_GPIO_InitPin_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.GPIO_InitPin_CallInstance = CMock_Guts_MemChain(Mock.GPIO_InitPin_CallInstance, cmock_guts_index);
  Mock.GPIO_InitPin_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->ReturnVal = cmock_to_return;
  Mock.GPIO_InitPin_IgnoreBool = (char)1;
}

void GPIO_InitPin_CMockStopIgnore(void)
{
  if(Mock.GPIO_InitPin_IgnoreBool)
    Mock.GPIO_InitPin_CallInstance = CMock_Guts_MemNext(Mock.GPIO_InitPin_CallInstance);
  Mock.GPIO_InitPin_IgnoreBool = (char)0;
}

void GPIO_InitPin_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, GPIO_CFG_t* GPIO_Config, ErrorStatus_t cmock_to_return)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_GPIO_InitPin_CALL_INSTANCE));
  CMOCK_GPIO_InitPin_CALL_INSTANCE* cmock_call_instance = (CMOCK_GPIO_InitPin_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.GPIO_InitPin_CallInstance = CMock_Guts_MemChain(Mock.GPIO_InitPin_CallInstance, cmock_guts_index);
  Mock.GPIO_InitPin_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_GPIO_InitPin(cmock_call_instance, GPIO_Config);
  memcpy((void*)(&cmock_call_instance->ReturnVal), (void*)(&cmock_to_return),
         sizeof(ErrorStatus_t[sizeof(cmock_to_return) == sizeof(ErrorStatus_t) ? 1 : -1])); /* add ErrorStatus_t to :treat_as_array if this causes an error */
}

void GPIO_InitPin_AddCallback(CMOCK_GPIO_InitPin_CALLBACK Callback)
{
  Mock.GPIO_InitPin_IgnoreBool = (char)0;
  Mock.GPIO_InitPin_CallbackBool = (char)1;
  Mock.GPIO_InitPin_CallbackFunctionPointer = Callback;
}

void GPIO_InitPin_Stub(CMOCK_GPIO_InitPin_CALLBACK Callback)
{
  Mock.GPIO_InitPin_IgnoreBool = (char)0;
  Mock.GPIO_InitPin_CallbackBool = (char)0;
  Mock.GPIO_InitPin_CallbackFunctionPointer = Callback;
}

ErrorStatus_t GPIO_SetPinValue(void* GPIO_Port, uint32_t GPIO_Pin, uint32_t GPIO_PinState)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_GPIO_SetPinValue_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_GPIO_SetPinValue);
  cmock_call_instance = (CMOCK_GPIO_SetPinValue_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.GPIO_SetPinValue_CallInstance);
  Mock.GPIO_SetPinValue_CallInstance = CMock_Guts_MemNext(Mock.GPIO_SetPinValue_CallInstance);
  if (Mock.GPIO_SetPinValue_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    if (cmock_call_instance == NULL)
      return Mock.GPIO_SetPinValue_FinalReturn;
    memcpy((void*)(&Mock.GPIO_SetPinValue_FinalReturn), (void*)(&cmock_call_instance->ReturnVal),
         sizeof(ErrorStatus_t[sizeof(cmock_call_instance->ReturnVal) == sizeof(ErrorStatus_t) ? 1 : -1])); /* add ErrorStatus_t to :treat_as_array if this causes an error */
    return cmock_call_instance->ReturnVal;
  }
  if (!Mock.GPIO_SetPinValue_CallbackBool &&
      Mock.GPIO_SetPinValue_CallbackFunctionPointer != NULL)
  {
    ErrorStatus_t cmock_cb_ret = Mock.GPIO_SetPinValue_CallbackFunctionPointer(GPIO_Port, GPIO_Pin, GPIO_PinState, Mock.GPIO_SetPinValue_CallbackCalls++);
    UNITY_CLR_DETAILS();
    return cmock_cb_ret;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_GPIO_SetPinValue,CMockString_GPIO_Port);
    if (cmock_call_instance->Expected_GPIO_Port == NULL)
      { UNITY_TEST_ASSERT_NULL(GPIO_Port, cmock_line, CMockStringExpNULL); }
    else
      { UNITY_TEST_ASSERT_EQUAL_HEX8_ARRAY(cmock_call_instance->Expected_GPIO_Port, GPIO_Port, 1, cmock_line, CMockStringMismatch); }
  }
  {
    UNITY_SET_DETAILS(CMockString_GPIO_SetPinValue,CMockString_GPIO_Pin);
    UNITY_TEST_ASSERT_EQUAL_HEX32(cmock_call_instance->Expected_GPIO_Pin, GPIO_Pin, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_GPIO_SetPinValue,CMockString_GPIO_PinState);
    UNITY_TEST_ASSERT_EQUAL_HEX32(cmock_call_instance->Expected_GPIO_PinState, GPIO_PinState, cmock_line, CMockStringMismatch);
  }
  if (Mock.GPIO_SetPinValue_CallbackFunctionPointer != NULL)
  {
    cmock_call_instance->ReturnVal = Mock.GPIO_SetPinValue_CallbackFunctionPointer(GPIO_Port, GPIO_Pin, GPIO_PinState, Mock.GPIO_SetPinValue_CallbackCalls++);
  }
  UNITY_CLR_DETAILS();
  return cmock_call_instance->ReturnVal;
}

void CMockExpectParameters_GPIO_SetPinValue(CMOCK_GPIO_SetPinValue_CALL_INSTANCE* cmock_call_instance, void* GPIO_Port, uint32_t GPIO_Pin, uint32_t GPIO_PinState);
void CMockExpectParameters_GPIO_SetPinValue(CMOCK_GPIO_SetPinValue_CALL_INSTANCE* cmock_call_instance, void* GPIO_Port, uint32_t GPIO_Pin, uint32_t GPIO_PinState)
{
  cmock_call_instance->Expected_GPIO_Port = GPIO_Port;
  cmock_call_instance->Expected_GPIO_Pin = GPIO_Pin;
  cmock_call_instance->Expected_GPIO_PinState = GPIO_PinState;
}

void GPIO_SetPinValue_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, ErrorStatus_t cmock_to_return)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_GPIO_SetPinValue_CALL_INSTANCE));
  CMOCK_GPIO_SetPinValue_CALL_INSTANCE* cmock_call_instance = (CMOCK_GPIO_SetPinValue_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.GPIO_SetPinValue_CallInstance = CMock_Guts_MemChain(Mock.GPIO_SetPinValue_CallInstance, cmock_guts_index);
  Mock.GPIO_SetPinValue_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->ReturnVal = cmock_to_return;
  Mock.GPIO_SetPinValue_IgnoreBool = (char)1;
}

void GPIO_SetPinValue_CMockStopIgnore(void)
{
  if(Mock.GPIO_SetPinValue_IgnoreBool)
    Mock.GPIO_SetPinValue_CallInstance = CMock_Guts_MemNext(Mock.GPIO_SetPinValue_CallInstance);
  Mock.GPIO_SetPinValue_IgnoreBool = (char)0;
}

void GPIO_SetPinValue_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, void* GPIO_Port, uint32_t GPIO_Pin, uint32_t GPIO_PinState, ErrorStatus_t cmock_to_return)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_GPIO_SetPinValue_CALL_INSTANCE));
  CMOCK_GPIO_SetPinValue_CALL_INSTANCE* cmock_call_instance = (CMOCK_GPIO_SetPinValue_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.GPIO_SetPinValue_CallInstance = CMock_Guts_MemChain(Mock.GPIO_SetPinValue_CallInstance, cmock_guts_index);
  Mock.GPIO_SetPinValue_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_GPIO_SetPinValue(cmock_call_instance, GPIO_Port, GPIO_Pin, GPIO_PinState);
  memcpy((void*)(&cmock_call_instance->ReturnVal), (void*)(&cmock_to_return),
         sizeof(ErrorStatus_t[sizeof(cmock_to_return) == sizeof(ErrorStatus_t) ? 1 : -1])); /* add ErrorStatus_t to :treat_as_array if this causes an error */
}

void GPIO_SetPinValue_AddCallback(CMOCK_GPIO_SetPinValue_CALLBACK Callback)
{
  Mock.GPIO_SetPinValue_IgnoreBool = (char)0;
  Mock.GPIO_SetPinValue_CallbackBool = (char)1;
  Mock.GPIO_SetPinValue_CallbackFunctionPointer = Callback;
}

void GPIO_SetPinValue_Stub(CMOCK_GPIO_SetPinValue_CALLBACK Callback)
{
  Mock.GPIO_SetPinValue_IgnoreBool = (char)0;
  Mock.GPIO_SetPinValue_CallbackBool = (char)0;
  Mock.GPIO_SetPinValue_CallbackFunctionPointer = Callback;
}

ErrorStatus_t GPIO_GetPinValue(void* GPIO_Port, uint32_t GPIO_Pin, uint32_t* GPIO_PinState)
{
  UNITY_LINE_TYPE cmock_line = TEST_LINE_NUM;
  CMOCK_GPIO_GetPinValue_CALL_INSTANCE* cmock_call_instance;
  UNITY_SET_DETAIL(CMockString_GPIO_GetPinValue);
  cmock_call_instance = (CMOCK_GPIO_GetPinValue_CALL_INSTANCE*)CMock_Guts_GetAddressFor(Mock.GPIO_GetPinValue_CallInstance);
  Mock.GPIO_GetPinValue_CallInstance = CMock_Guts_MemNext(Mock.GPIO_GetPinValue_CallInstance);
  if (Mock.GPIO_GetPinValue_IgnoreBool)
  {
    UNITY_CLR_DETAILS();
    if (cmock_call_instance == NULL)
      return Mock.GPIO_GetPinValue_FinalReturn;
    memcpy((void*)(&Mock.GPIO_GetPinValue_FinalReturn), (void*)(&cmock_call_instance->ReturnVal),
         sizeof(ErrorStatus_t[sizeof(cmock_call_instance->ReturnVal) == sizeof(ErrorStatus_t) ? 1 : -1])); /* add ErrorStatus_t to :treat_as_array if this causes an error */
    return cmock_call_instance->ReturnVal;
  }
  if (!Mock.GPIO_GetPinValue_CallbackBool &&
      Mock.GPIO_GetPinValue_CallbackFunctionPointer != NULL)
  {
    ErrorStatus_t cmock_cb_ret = Mock.GPIO_GetPinValue_CallbackFunctionPointer(GPIO_Port, GPIO_Pin, GPIO_PinState, Mock.GPIO_GetPinValue_CallbackCalls++);
    UNITY_CLR_DETAILS();
    return cmock_cb_ret;
  }
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringCalledMore);
  cmock_line = cmock_call_instance->LineNumber;
  if (cmock_call_instance->CallOrder > ++GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledEarly);
  if (cmock_call_instance->CallOrder < GlobalVerifyOrder)
    UNITY_TEST_FAIL(cmock_line, CMockStringCalledLate);
  {
    UNITY_SET_DETAILS(CMockString_GPIO_GetPinValue,CMockString_GPIO_Port);
    if (cmock_call_instance->Expected_GPIO_Port == NULL)
      { UNITY_TEST_ASSERT_NULL(GPIO_Port, cmock_line, CMockStringExpNULL); }
    else
      { UNITY_TEST_ASSERT_EQUAL_HEX8_ARRAY(cmock_call_instance->Expected_GPIO_Port, GPIO_Port, 1, cmock_line, CMockStringMismatch); }
  }
  {
    UNITY_SET_DETAILS(CMockString_GPIO_GetPinValue,CMockString_GPIO_Pin);
    UNITY_TEST_ASSERT_EQUAL_HEX32(cmock_call_instance->Expected_GPIO_Pin, GPIO_Pin, cmock_line, CMockStringMismatch);
  }
  {
    UNITY_SET_DETAILS(CMockString_GPIO_GetPinValue,CMockString_GPIO_PinState);
    if (cmock_call_instance->Expected_GPIO_PinState == NULL)
      { UNITY_TEST_ASSERT_NULL(GPIO_PinState, cmock_line, CMockStringExpNULL); }
    else
      { UNITY_TEST_ASSERT_EQUAL_HEX32_ARRAY(cmock_call_instance->Expected_GPIO_PinState, GPIO_PinState, 1, cmock_line, CMockStringMismatch); }
  }
  if (Mock.GPIO_GetPinValue_CallbackFunctionPointer != NULL)
  {
    cmock_call_instance->ReturnVal = Mock.GPIO_GetPinValue_CallbackFunctionPointer(GPIO_Port, GPIO_Pin, GPIO_PinState, Mock.GPIO_GetPinValue_CallbackCalls++);
  }
  UNITY_CLR_DETAILS();
  return cmock_call_instance->ReturnVal;
}

void CMockExpectParameters_GPIO_GetPinValue(CMOCK_GPIO_GetPinValue_CALL_INSTANCE* cmock_call_instance, void* GPIO_Port, uint32_t GPIO_Pin, uint32_t* GPIO_PinState);
void CMockExpectParameters_GPIO_GetPinValue(CMOCK_GPIO_GetPinValue_CALL_INSTANCE* cmock_call_instance, void* GPIO_Port, uint32_t GPIO_Pin, uint32_t* GPIO_PinState)
{
  cmock_call_instance->Expected_GPIO_Port = GPIO_Port;
  cmock_call_instance->Expected_GPIO_Pin = GPIO_Pin;
  cmock_call_instance->Expected_GPIO_PinState = GPIO_PinState;
}

void GPIO_GetPinValue_CMockIgnoreAndReturn(UNITY_LINE_TYPE cmock_line, ErrorStatus_t cmock_to_return)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_GPIO_GetPinValue_CALL_INSTANCE));
  CMOCK_GPIO_GetPinValue_CALL_INSTANCE* cmock_call_instance = (CMOCK_GPIO_GetPinValue_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.GPIO_GetPinValue_CallInstance = CMock_Guts_MemChain(Mock.GPIO_GetPinValue_CallInstance, cmock_guts_index);
  Mock.GPIO_GetPinValue_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->ReturnVal = cmock_to_return;
  Mock.GPIO_GetPinValue_IgnoreBool = (char)1;
}

void GPIO_GetPinValue_CMockStopIgnore(void)
{
  if(Mock.GPIO_GetPinValue_IgnoreBool)
    Mock.GPIO_GetPinValue_CallInstance = CMock_Guts_MemNext(Mock.GPIO_GetPinValue_CallInstance);
  Mock.GPIO_GetPinValue_IgnoreBool = (char)0;
}

void GPIO_GetPinValue_CMockExpectAndReturn(UNITY_LINE_TYPE cmock_line, void* GPIO_Port, uint32_t GPIO_Pin, uint32_t* GPIO_PinState, ErrorStatus_t cmock_to_return)
{
  CMOCK_MEM_INDEX_TYPE cmock_guts_index = CMock_Guts_MemNew(sizeof(CMOCK_GPIO_GetPinValue_CALL_INSTANCE));
  CMOCK_GPIO_GetPinValue_CALL_INSTANCE* cmock_call_instance = (CMOCK_GPIO_GetPinValue_CALL_INSTANCE*)CMock_Guts_GetAddressFor(cmock_guts_index);
  UNITY_TEST_ASSERT_NOT_NULL(cmock_call_instance, cmock_line, CMockStringOutOfMemory);
  memset(cmock_call_instance, 0, sizeof(*cmock_call_instance));
  Mock.GPIO_GetPinValue_CallInstance = CMock_Guts_MemChain(Mock.GPIO_GetPinValue_CallInstance, cmock_guts_index);
  Mock.GPIO_GetPinValue_IgnoreBool = (char)0;
  cmock_call_instance->LineNumber = cmock_line;
  cmock_call_instance->CallOrder = ++GlobalExpectCount;
  CMockExpectParameters_GPIO_GetPinValue(cmock_call_instance, GPIO_Port, GPIO_Pin, GPIO_PinState);
  memcpy((void*)(&cmock_call_instance->ReturnVal), (void*)(&cmock_to_return),
         sizeof(ErrorStatus_t[sizeof(cmock_to_return) == sizeof(ErrorStatus_t) ? 1 : -1])); /* add ErrorStatus_t to :treat_as_array if this causes an error */
}

void GPIO_GetPinValue_AddCallback(CMOCK_GPIO_GetPinValue_CALLBACK Callback)
{
  Mock.GPIO_GetPinValue_IgnoreBool = (char)0;
  Mock.GPIO_GetPinValue_CallbackBool = (char)1;
  Mock.GPIO_GetPinValue_CallbackFunctionPointer = Callback;
}

void GPIO_GetPinValue_Stub(CMOCK_GPIO_GetPinValue_CALLBACK Callback)
{
  Mock.GPIO_GetPinValue_IgnoreBool = (char)0;
  Mock.GPIO_GetPinValue_CallbackBool = (char)0;
  Mock.GPIO_GetPinValue_CallbackFunctionPointer = Callback;
}

