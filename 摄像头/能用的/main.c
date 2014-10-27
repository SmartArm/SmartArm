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

           /* READ_FIFO_PIXEL(Camera_Data);		//从FIFO读出一个rgb565像素到Camera_Data变量
           // uint8_t ch;
            temp=Camera_Data;
            ch=(uint8_t)(temp>>8);
            USART_SendData(USART1, ch);
	    while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//发送高8位
            temp=Camera_Data;
            ch=(u8) temp;
            USART_SendData(USART1, ch);
	    while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//发送低8位
            */
          FIFO_RCLK_L();
          ch=(uint8_t)GPIOB->IDR&0x00ff;
          USART_SendData(USART1, ch);
	  while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//发送高8位
          FIFO_RCLK_H();
          FIFO_RCLK_L();
          ch=(uint8_t)GPIOB->IDR&0x00ff;
          USART_SendData(USART1, ch);
	  while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//发送低8位
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
    //while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	//发送结束符
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
		   	FIFO_PREPARE;  			/*FIFO准备*/
            Get_imag_and_discor();	/*采集并显示*/           
			//Frame_Count++;			/*帧计数器加1*/	
			OV7670_VSYNC = 0;
           //EXTI->IMR |=(1<<4);  //开场中断
		
        }
    }
}

/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
