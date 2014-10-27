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
#include<stm32f4xx.h>
#include "OV7670.h"
#include "SysTick.h"	
#include "usart1.h"
#include "SCCB.h"

//#define READ_FIFO_PIXEL(RGB565)   	do{RGB565=0; FIFO_RCLK_L();RGB565 = (GPIOB->IDR <<8) & 0xff00;FIFO_RCLK_H();FIFO_RCLK_L();RGB565 |= (GPIOB->IDR) & 0x00ff;FIFO_RCLK_H();}while(0)
#define FIFO_PREPARE                do{FIFO_RRST_L();FIFO_RCLK_L();FIFO_RCLK_H();FIFO_RRST_H();FIFO_RCLK_L();FIFO_RCLK_H();}while(0)
uint8_t ch;
uint16_t temp;

void Get_imag_and_discor(void)
{

    u16 i, j;
    u16 Camera_Data;
    for(i = 0; i < 240; i++)
    {
        for(j = 0; j < 320; j++)
        {

           /* READ_FIFO_PIXEL(Camera_Data);		//��FIFO����һ��rgb565���ص�Camera_Data����
           // uint8_t ch;
            temp=Camera_Data;
            ch=(uint8_t)(temp>>8);
            USART_SendData(USART1, ch);
	    while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//���͸�8λ
            temp=Camera_Data;
            ch=(u8) temp;
            USART_SendData(USART1, ch);
	    while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//���͵�8λ
            */
          FIFO_RCLK_L();
          ch=(uint8_t)GPIOB->IDR&0x00ff;
          USART_SendData(USART1, ch);
	  while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//���͸�8λ
          FIFO_RCLK_H();
          FIFO_RCLK_L();
          ch=(uint8_t)GPIOB->IDR&0x00ff;
          USART_SendData(USART1, ch);
	  while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//���͵�8λ
          FIFO_RCLK_H();
          
        }
        //READ_FIFO_PIXEL(Camera_Data);
        /* 
         FIFO_RCLK_L();
          ch=(u8)GPIOB->IDR&0x00ff;
          FIFO_RCLK_H();
          FIFO_RCLK_L();
          ch=(u8)GPIOB->IDR&0x00ff;
         FIFO_RCLK_H();
       */
    }
    //USART_SendData(USART1, 0xff);
    //while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//���ͽ�����
}


int main(void) 	
{
   
        SysTick_Init();
        USART1_Config();
	SCCB_GPIO_Configuration();
        FIFO_GPIO_Configuration();
       while(OV7670_Init() != SUCCESS);
	VSYNC_Init();
        OV7670_VSYNC = 0;
        
    while(1)
    {
        if(OV7670_VSYNC==2)  // 1
        {	  
			Delay_ms(1);		
		   	FIFO_PREPARE;  			/*FIFO׼��*/
            Get_imag_and_discor();	/*�ɼ�����ʾ*/           
			//Frame_Count++;			/*֡��������1*/	
			OV7670_VSYNC = 0;
           //EXTI->IMR |=(1<<4);  //�����ж�
		
        }
    }
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
