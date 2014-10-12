#include "stm32f4xx_exti.h"
#include "SCCB.h"
#include "OV7670.h"
#include "systick.h"
#include "misc.h"
#include "SensorConfig.h"
#include "usart1.h"

#define Delay_ms(time)  
#define OV7670_REG_NUM 114
u8 volatile OV7670_VSYNC;	 /* ֡ͬ���ź� */
volatile u8 Frame_Count = 0 ;

extern Register_Info Sensor_Config[];






/************************************************
 * ��������Sensor_Init
 * ����  ��Sensor��ʼ��
 * ����  ����
 * ���  ������1�ɹ�������0ʧ��
 * ע��  ����
 ************************************************/
ErrorStatus OV7670_Init(void)
{
    u16 i = 0;
    u8 Sensor_IDCode = 0;
	printf("\r\n>>>>>>STM32F103 OV7670&FIFO Demo<<<<<<\r\n");
	//DEBUG("OV7670 Register Config Start!");
    if( 0 == SCCB_WriteByte ( 0x12, 0x80 ) ) /*��λsensor */
    {
        //DEBUG("����:SCCBд���ݴ���");
      printf("\r\n>>>>>>����:SCCBд���ݴ���<<<<<<\r\n");
        Delay_ms(50);
        return ERROR ;
    }
    Delay_ms(50);
    if( 0 == SCCB_ReadByte( &Sensor_IDCode,  0x0b ) )	 /* ��ȡsensor ID��*/
    {        
        //DEBUG("����:��ȡIDʧ��"); 
       printf("\r\n>>>>>>����:��ȡIDʧ��<<<<<<\r\n");
		return ERROR;

    }
    //DEBUG("Get ID success��SENSOR ID is 0x%x", Sensor_IDCode);
    //DEBUG("Config Register Number is %d ", OV7670_REG_NUM);
    //if(Sensor_IDCode == OV7670_ID)
   // {
        for( i = 0 ; i < OV7670_REG_NUM ; i++ )
        {
            if( 0 == SCCB_WriteByte(Sensor_Config[i].Address, Sensor_Config[i].Value) )
            {                
              printf("\r\n>>>>>>����:д�Ĵ���0x%xʧ��<<<<<<\r\n");
				//DEBUG("����:д�Ĵ���0x%xʧ��", Sensor_Config[i].Address);
				return ERROR;
            }
        }
  //  }
   // else
   // {
   //     return ERROR;
   // }
	//DEBUG("OV7670 Register Config Success!");
    return SUCCESS;
}
void XCLK_Config(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  /* Configure MCO pin(PA8) in alternate function */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  /* System clock selected to output on MCO1 pin(PA8)*/
  RCC_MCO1Config  (RCC_MCO1Source_HSE,RCC_MCO1Div_1);

}//�ݲ���

