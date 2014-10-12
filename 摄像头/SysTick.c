
#include "SysTick.h"

static __IO u32 TimingDelay;
static __IO u32 RunTime=0;
/*
 * 函数名：SysTick_Init
 * 描述  ：启动系统滴答定时器 SysTick
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用 
 */
void SysTick_Init(void)
{
	/* SystemCoreClock / 1000    1ms中断一次
	 * SystemCoreClock / 100000	 10us中断一次
	 * SystemCoreClock / 1000000 1us中断一次
	 */

	if (SysTick_Config(SystemCoreClock / 1000 ))	// ST3.5.0库版本
	{ 
		/* Capture error */ 
		while (1);
	}
		// 关闭滴答定时器  
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
        return;
}


/*
 * 函数名：Delay_ms
 * 描述  ：ms延时程序,1ms为一个单位
 * 输入  ：- nTime
 * 输出  ：无
 * 调用  ：Delay_ms( 1 ) 则实现的延时为 1 * 1ms = 1ms
 *       ：外部调用 
 */

void Delay_ms(__IO uint32_t nTime)
{ 
	TimingDelay = nTime;	

	// 使能滴答定时器  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay != 0);
		// 关闭滴答定时器  
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}
uint32_t getTime(void){
  return RunTime;
}

/*
 * 函数名：TimingDelay_Decrement
 * 描述  ：获取节拍程序
 * 输入  ：无
 * 输出  ：无
 * 调用  ：在 SysTick 中断函数 SysTick_Handler()调用
 */  
void TimingDelay_Decrement(void)
{
  RunTime++;//每次中断时该变量+1
	if (TimingDelay != 0x00)
	{ 
	TimingDelay--;
	}
}

