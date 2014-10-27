#ifndef __SCCB_H
#define __SCCB_H

#include<stm32f4xx.h>
#define SCL_H         GPIO_SetBits(GPIOC , GPIO_Pin_14) 
#define SCL_L         GPIO_ResetBits(GPIOC , GPIO_Pin_14) 
   
#define SDA_H         GPIO_SetBits(GPIOC , GPIO_Pin_15) 
#define SDA_L         GPIO_ResetBits(GPIOC , GPIO_Pin_15) 

#define SCL_read      GPIO_ReadInputDataBit(GPIOC , GPIO_Pin_14) 
#define SDA_read      GPIO_ReadInputDataBit(GPIOC , GPIO_Pin_15) 

#define ADDR_OV7725   0x42

void SCCB_GPIO_Configuration(void);
int SCCB_WriteByte( u16 WriteAddress , u8 SendByte);
int SCCB_ReadByte(u8* pBuffer,   u8 ReadAddress);

#endif 
