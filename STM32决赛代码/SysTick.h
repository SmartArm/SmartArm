#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f4xx.h"

void SysTick_Init(void);
void Delay_us(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
uint32_t getTime(void);
#endif /* __SYSTICK_H */
