#ifndef __OV7670_H
#define __OV7670_H 

	   
#include<stm32f4xx.h>
 


#define OV7670_ID       0x73

//#define FIFO_CS_H()     GPIOD->BSRR =GPIO_Pin_6	  
//#define FIFO_CS_L()     GPIOD->BRR  =GPIO_Pin_6	  /*����ʹFIFO���ʹ��*/

#define FIFO_WRST_H()   GPIO_SetBits(GPIOC , GPIO_Pin_2) 	  /*��������FIFOд(����from����ͷ)ָ���˶� */
#define FIFO_WRST_L()   GPIO_ResetBits(GPIOC , GPIO_Pin_2) 	  /*����ʹFIFOд(����from����ͷ)ָ�븴λ*/

#define FIFO_RRST_H()   GPIO_SetBits(GPIOC , GPIO_Pin_4) 	  /*��������FIFO��(���ݴ�FIFO���)ָ���˶� */
#define FIFO_RRST_L()   GPIO_ResetBits(GPIOC , GPIO_Pin_4) 	  /*����ʹFIFO��(���ݴ�FIFO���)ָ�븴λ */

#define FIFO_RCLK_H()   GPIO_SetBits(GPIOC , GPIO_Pin_5)	  
#define FIFO_RCLK_L()   GPIO_ResetBits(GPIOC , GPIO_Pin_5)	  /*FIFO�������ʱ��*/

#define FIFO_WE_H()     GPIO_SetBits(GPIOC , GPIO_Pin_3) 	  /*����ʹFIFOд����*/
#define FIFO_WE_L()     GPIO_ResetBits(GPIOC , GPIO_Pin_3)	  



extern u8 volatile OV7670_VSYNC;

ErrorStatus OV7670_Init(void);

void VSYNC_Init(void);
void FIFO_GPIO_Configuration(void);


#endif























