/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * �ļ���  ��main.c
 * ����    ��OV7670����ͷ����         
 * ʵ��ƽ̨��Ұ��STM32������
 * ��汾  ��ST3.5.0
 *
 * ����    ��wildfire team 
 * ��̳    ��www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
 * �Ա�    ��http://firestm32.taobao.com
**********************************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#include "OV7670.h"

#include "SysTick.h"	

#include "SCCB.h"
#include "pwm.h"
#include "usart1.h"


/*����ͷ�ɼ�*/
#define R 120//��
#define C 160//��
uint16_t Data1[R][C];
uint8_t  imageOld[R][C];
uint8_t  InterruptLine=0;
uint8_t  firstByte=1;//���ж�������ͷ�ĸ�8λ���ݻ��ǵͰ�λ����
uint8_t  j=0;          //ÿ�еĵ�j�����ص�
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
    TIM2_PWM_Init();//����ʱ���ź�
    SysTick_Init();
    USART1_NVIC_Config();//���ô����ж����ȼ�
    USART_Config();//���ô���1
    
	
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
     /* EXTI->IMR &=(0<<0);  //���������ж�
      EXTI->IMR &=(0<<1);  //�������ж�
      EXTI->IMR &=(0<<4);  //���γ��ж�
      EXTI->PR |=(1<<4);  //������ұ��λ��
      EXTI->PR |=(1<<0);  //������ұ��λ��
      EXTI->PR |=(1<<1);  //������ұ��λ��
*/
        for(int x=0;x<120;x++)
        {
          for(int y=0;y<160;y++)
          {
            unsigned char ch;
            u16 temp=Data1[x][y];
            ch=(u8)(temp>>8);
            USART_SendData(USART1, ch);
	    while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//���͸�8λ
            temp=Data1[x][y];
            ch=(u8) temp;
            USART_SendData(USART1, ch);
	    while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//���͵�8λ
          }
        }
        EXTI->IMR |=(1<<4);  //�����ж�
        EXTI->IMR |=(1<<1);  //�����ж�
        EXTI->IMR |=(1<<0);  //�������ж�
        DataReadyFlag=0;
      }
    }
}
//���жϣ������жϣ����жϣ�����ͷ�������ŵ�GPIO����
void GPIO_Config(){
      GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
   /* gpio ���ã�����HREF���ж�ΪPC1,���ص��ж�ΪPC0*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    //ѡ���ⲿ�ж�Դ 
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
    
  /* gpio ���ã�����VSYNC���ж�ΪPA4*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);
    
/*����ͷ���ݽӿ�PB0-PB7 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//�ж����ȼ�����
void NVIC_Config(void){
  NVIC_InitTypeDef NVIC_InitStructure;
   /*���������жϺ����жϴ���************************************/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  
  
  /*���ڳ��ж�************************************/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
  
}
/*****************************************************************************
���ܣ��ⲿ�жϺ���
���룺none
�����none
*****************************************************************************/
void EXTI_Config(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  
  /* ���ж� */
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&EXTI_InitStructure);
  /* ���ж� */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_Init(&EXTI_InitStructure);
  /* �����ж� */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_Init(&EXTI_InitStructure);
}
/*****************************************************************************
���ܣ����ж��жϺ���
���룺none
�����none
*****************************************************************************/
void EXTI4_IRQHandler()
{
  //printf("\r\n>>>>>>���ж�<<<<<<\r\n");
  //******************���ж�********************************************
   if(EXTI_GetITStatus(EXTI_Line4) != RESET)
 {
    EXTI->PR |=(1<<4);  //������ұ��λ��
    
    EXTI->IMR &=(0<<0);  //���������ж�
      EXTI->IMR &=(0<<1);  //�������ж�
      EXTI->IMR &=(0<<4);  //���γ��ж�
      EXTI->PR |=(1<<4);  //������ұ��λ��
      EXTI->PR |=(1<<0);  //������ұ��λ��
      EXTI->PR |=(1<<1);  //������ұ��λ��
      
    InterruptLine = 0;  //ͼ��������α�����
    DataReadyFlag= 1; //����1�е�ͼ��׼�����
    
    
 }
}
/*****************************************************************************
���ܣ������ж��жϺ���
���룺none
�����none
*****************************************************************************/
void EXTI0_IRQHandler()
{
//printf("\r\n>>>>>>�����ж�<<<<<<\r\n");
 if(j<160)
 {
 //**************** �����ж� ***********************************************
 if(EXTI_GetITStatus(EXTI_Line0) != RESET)
 {
    EXTI->PR |=(1<<0);   //������ұ��λ��
     if(firstByte==1){
     Data1[InterruptLine][j]=(((uint16_t)GPIOB->IDR)&0x00ff);   //����8λ����
     Data1[InterruptLine][j]= Data1[InterruptLine][j]*256;
     firstByte=0;
     }
     else
     {
       Data1[InterruptLine][j]+=(((uint16_t)GPIOB->IDR)&0x00ff); //����8λ����
       j++;//��һ�����ص�
       firstByte=1;
     }
 }
 }
}
 /*****************************************************************************
���ܣ����ж��жϺ���
���룺none
�����none
*****************************************************************************/
void EXTI1_IRQHandler()
{
//printf("\r\n>>>>>>���ж�<<<<<<\r\n");
 if(InterruptLine<120)
 {
  //**************** ���ж� ***********************************************
 if(EXTI_GetITStatus(EXTI_Line1) != RESET)
 {
   EXTI->PR |=(1<<1);   //������ұ��λ��
   
   if(DataReadyFlag == 1)
     return;
   j=0;
   InterruptLine++;

 }
 }

 }
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/




