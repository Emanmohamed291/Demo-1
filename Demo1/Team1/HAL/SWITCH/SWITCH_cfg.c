
#include "MCAL/01_GPIO/GPIO.h"
#include "SWITCH.h"

/*** SWITCHS user configuration ***/
const SWITCH_cfg_t SWITCHS[_SWITCH_NUM]={
	[SWITCH_MODE]={.SWITCH_Port=GPIO_A, .SWITCH_Pin=GPIO_PIN3,  .SWITCH_Mode=SWITCH_INPUT_PU},
	[SWITCH_UP]={.SWITCH_Port=GPIO_A, .SWITCH_Pin=GPIO_PIN4,  .SWITCH_Mode=SWITCH_INPUT_PU},
	// [SWITCH_Ok]={.SWITCH_Port=GPIO_A, .SWITCH_Pin=GPIO_PIN5,  .SWITCH_Mode=SWITCH_INPUT_PU},
	// [SWITCH_EDIT]={.SWITCH_Port=GPIO_A, .SWITCH_Pin=GPIO_PIN6,  .SWITCH_Mode=SWITCH_INPUT_PU},
	// [SWITCH_STARTSTOP_STOPWATCH]={.SWITCH_Port=GPIO_A, .SWITCH_Pin=GPIO_PIN7,  .SWITCH_Mode=SWITCH_INPUT_PU},
	// [SWITCH_LEFT_RESET_STOPWATCH]={.SWITCH_Port=GPIO_A, .SWITCH_Pin=GPIO_PIN8,  .SWITCH_Mode=SWITCH_INPUT_PU},
	// [SWITCH_DOWN_PAUSECONT_STOPWATCH]={.SWITCH_Port=GPIO_A, .SWITCH_Pin=GPIO_PIN11,  .SWITCH_Mode=SWITCH_INPUT_PU},
	// [SWITCH_RIGHT]={.SWITCH_Port=GPIO_A, .SWITCH_Pin=GPIO_PIN12,  .SWITCH_Mode=SWITCH_INPUT_PU}
};