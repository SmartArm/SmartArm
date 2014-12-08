#include "Timer.h"

/*
 * 函数名：TIM4_Config
 * 描述  ：TIM4做定时中断
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
void TIM4_Config(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  /* Time base configuration */    
  TIM_TimeBaseStructure.TIM_Period = 1000;       //为一个定时周期0.1s
  TIM_TimeBaseStructure.TIM_Prescaler = 16799;     //设置预分频,10KHZ 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;  //设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_Trigger,ENABLE);//使能溢出中断

  TIM_ARRPreloadConfig(TIM4, ENABLE);      // 使能TIM2重载寄存器ARR

  /* TIM2 enable counter */
  TIM_Cmd(TIM4, ENABLE);                   //使能定时器4
  TIM4_NVIC_Configuare();
}
//NVIC设置 系统中断管理
void TIM4_NVIC_Configuare(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannel  = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        
    NVIC_Init(&NVIC_InitStructure);//TIM2 configuration 
}
