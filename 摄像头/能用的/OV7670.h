#ifndef __OV7670_H
#define __OV7670_H 

	   
#include<stm32f4xx.h>
 


#define OV7670_ID       0x73

//#define FIFO_CS_H()     GPIOD->BSRR =GPIO_Pin_6	  
//#define FIFO_CS_L()     GPIOD->BRR  =GPIO_Pin_6	  /*拉低使FIFO输出使能*/

#define FIFO_WRST_H()   GPIO_SetBits(GPIOC , GPIO_Pin_2) 	  /*拉高允许FIFO写(数据from摄像头)指针运动 */
#define FIFO_WRST_L()   GPIO_ResetBits(GPIOC , GPIO_Pin_2) 	  /*拉低使FIFO写(数据from摄像头)指针复位*/

#define FIFO_RRST_H()   GPIO_SetBits(GPIOC , GPIO_Pin_4) 	  /*拉高允许FIFO读(数据从FIFO输出)指针运动 */
#define FIFO_RRST_L()   GPIO_ResetBits(GPIOC , GPIO_Pin_4) 	  /*拉低使FIFO读(数据从FIFO输出)指针复位 */

#define FIFO_RCLK_H()   GPIO_SetBits(GPIOC , GPIO_Pin_5)	  
#define FIFO_RCLK_L()   GPIO_ResetBits(GPIOC , GPIO_Pin_5)	  /*FIFO输出数据时钟*/

#define FIFO_WE_H()     GPIO_SetBits(GPIOC , GPIO_Pin_3) 	  /*拉高使FIFO写允许*/
#define FIFO_WE_L()     GPIO_ResetBits(GPIOC , GPIO_Pin_3)	  



extern u8 volatile OV7670_VSYNC;

ErrorStatus OV7670_Init(void);

void VSYNC_Init(void);
void FIFO_GPIO_Configuration(void);


#endif























