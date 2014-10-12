/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * 文件名  ：main.c
 * 描述    ：OV7670摄像头例程         
 * 实验平台：野火STM32开发板
 * 库版本  ：ST3.5.0
 *
 * 作者    ：wildfire team 
 * 论坛    ：www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
 * 淘宝    ：http://firestm32.taobao.com
**********************************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#include "OV7670.h"

#include "SysTick.h"	

#include "SCCB.h"
#include "pwm.h"
#include "usart1.h"


/*摄像头采集*/
#define R 120//行
#define C 160//列
uint16_t Data1[R][C];
uint8_t  imageOld[R][C];
uint8_t  InterruptLine=0;
uint8_t  firstByte=1;//【判断是摄像头的高8位数据还是低八位数据
uint8_t  j=0;          //每行的第j个像素点
uint8_t DataReadyFlag=0;
uint16_t * Data=&Data1[0][0];
void GPIO_Config(void);
void NVIC_Config(void);
void EXTI_Config(void);
int main(void) 	
{
    GPIO_Config();
    NVIC_Config();
    EXTI_Config();
    TIM2_PWM_Init();//用作时钟信号
    SysTick_Init();
    USART1_NVIC_Config();//配置串口中断优先级
    USART_Config();//配置串口1
    
	
    SCCB_GPIO_Configuration();
      //  XCLK_Config();
    
   while(OV7670_Init() != SUCCESS);
    //VSYNC_Init();    						
   
    //OV7670_VSYNC = 0;
    
   /* while(1){
       USART_SendData(USART1, 'a');
       while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	
    }
   */
	
    while(1)
    {
      if(DataReadyFlag==1){
     /* EXTI->IMR &=(0<<0);  //屏蔽像素中断
      EXTI->IMR &=(0<<1);  //屏蔽行中断
      EXTI->IMR &=(0<<4);  //屏蔽场中断
      EXTI->PR |=(1<<4);  //清除悬挂标记位；
      EXTI->PR |=(1<<0);  //清除悬挂标记位；
      EXTI->PR |=(1<<1);  //清除悬挂标记位；
*/
        for(int x=0;x<120;x++)
        {
          for(int y=0;y<160;y++)
          {
            unsigned char ch;
            u16 temp=Data1[x][y];
            ch=(u8)(temp>>8);
            USART_SendData(USART1, ch);
	    while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//发送高8位
            temp=Data1[x][y];
            ch=(u8) temp;
            USART_SendData(USART1, ch);
	    while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//发送低8位
          }
        }
        EXTI->IMR |=(1<<4);  //开场中断
        EXTI->IMR |=(1<<1);  //开行中断
        EXTI->IMR |=(1<<0);  //开像素中断
        DataReadyFlag=0;
      }
    }
}
//行中断，像素中断，场中断，摄像头数据引脚的GPIO配置
void GPIO_Config(){
      GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
   /* gpio 配置，配置HREF行中断为PC1,像素点中断为PC0*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    //选择外部中断源 
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
    
  /* gpio 配置，配置VSYNC场中断为PA4*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);
    
/*摄像头数据接口PB0-PB7 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//中断优先级配置
void NVIC_Config(void){
  NVIC_InitTypeDef NVIC_InitStructure;
   /*用于像素中断和行中断触发************************************/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  
  
  /*用于场中断************************************/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
  
}
/*****************************************************************************
功能：外部中断函数
输入：none
输出：none
*****************************************************************************/
void EXTI_Config(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  
  /* 场中断 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&EXTI_InitStructure);
  /* 行中断 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_Init(&EXTI_InitStructure);
  /* 像素中断 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_Init(&EXTI_InitStructure);
}
/*****************************************************************************
功能：场中断中断函数
输入：none
输出：none
*****************************************************************************/
void EXTI4_IRQHandler()
{
  //printf("\r\n>>>>>>场中断<<<<<<\r\n");
  //******************场中断********************************************
   if(EXTI_GetITStatus(EXTI_Line4) != RESET)
 {
    EXTI->PR |=(1<<4);  //清除悬挂标记位；
    
    EXTI->IMR &=(0<<0);  //屏蔽像素中断
      EXTI->IMR &=(0<<1);  //屏蔽行中断
      EXTI->IMR &=(0<<4);  //屏蔽场中断
      EXTI->PR |=(1<<4);  //清除悬挂标记位；
      EXTI->PR |=(1<<0);  //清除悬挂标记位；
      EXTI->PR |=(1<<1);  //清除悬挂标记位；
      
    InterruptLine = 0;  //图像的行数游标清零
    DataReadyFlag= 1; //数组1中的图像准备完毕
    
    
 }
}
/*****************************************************************************
功能：像素中断中断函数
输入：none
输出：none
*****************************************************************************/
void EXTI0_IRQHandler()
{
//printf("\r\n>>>>>>像素中断<<<<<<\r\n");
 if(j<160)
 {
 //**************** 像素中断 ***********************************************
 if(EXTI_GetITStatus(EXTI_Line0) != RESET)
 {
    EXTI->PR |=(1<<0);   //清除悬挂标记位；
     if(firstByte==1){
     Data1[InterruptLine][j]=(((uint16_t)GPIOB->IDR)&0x00ff);   //读高8位数据
     Data1[InterruptLine][j]= Data1[InterruptLine][j]*256;
     firstByte=0;
     }
     else
     {
       Data1[InterruptLine][j]+=(((uint16_t)GPIOB->IDR)&0x00ff); //读低8位数据
       j++;//下一个像素点
       firstByte=1;
     }
 }
 }
}
 /*****************************************************************************
功能：行中断中断函数
输入：none
输出：none
*****************************************************************************/
void EXTI1_IRQHandler()
{
//printf("\r\n>>>>>>行中断<<<<<<\r\n");
 if(InterruptLine<120)
 {
  //**************** 行中断 ***********************************************
 if(EXTI_GetITStatus(EXTI_Line1) != RESET)
 {
   EXTI->PR |=(1<<1);   //清除悬挂标记位；
   
   if(DataReadyFlag == 1)
     return;
   j=0;
   InterruptLine++;

 }
 }

 }
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/




