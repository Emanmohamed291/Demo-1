
#include "NVIC.h"
#include"stm32f401cc.h"


void EXTI0_IRQHandler()
{
   u8 x=0;
   u8 z=0;
   NVIC_GetActiveStatus(NVIC_EXT0_INTERRRUPT,&z);

	   x++;

}
int main()
{
	 u8 x=1;
    NVIC_GetActiveStatus(NVIC_EXT0_INTERRRUPT, &x);
    NVIC_ClearPending(NVIC_EXT0_INTERRRUPT);
    NVIC_EnableInterrupt(NVIC_EXT0_INTERRRUPT);
    NVIC_GetActiveStatus(NVIC_EXT0_INTERRRUPT, &x);
    NVIC_GenerateSWI(NVIC_EXT0_INTERRRUPT);
    NVIC_GetActiveStatus(NVIC_EXT0_INTERRRUPT, &x);
    while(1);

    return 0;
}
