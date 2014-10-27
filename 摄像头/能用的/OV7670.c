
#include "SCCB.h"
#include "OV7670.h"
#include "systick.h"
#include "misc.h"
#include "SensorConfig.h"
#include "usart1.h"


#define OV7670_REG_NUM 166
u8 volatile OV7670_VSYNC;	 /* ֡ͬ���ź� */
volatile u8 Frame_Count = 0 ;

extern Register_Info Sensor_Config[];



/************************************************
 * ��������FIFO_GPIO_Configuration
 * ����  ��FIFO GPIO����
 * ����  ����
 * ���  ����
 * ע��  ����
 ************************************************/
void FIFO_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE); 

    /*PC2(FIFO_WRST--FIFOд��λ)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*PC3(FIFO_WEN--FIFOдʹ��)  (FIFO_REN--FIFO��ʹ��ֱ�ӽӵ�)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*PC4(FIFO_RRST--FIFO����λ)  PC5(FIFO_RCLK-FIFO��ʱ��)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*PB0-PB7(FIFO_DATA--FIFO�������)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//Ĭ�ϼ��Ǹ�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //FIFO_CS_L();	  					/*����ʹFIFO���ʹ��*//����Ӳ���Ͻӵ���
    FIFO_WE_H();   						/*����ʹFIFOд����*/

}




/************************************************
 * ��������VSYNC_GPIO_Configuration
 * ����  ��OV7670 GPIO����
 * ����  ����
 * ���  ����
 * ע��  ����
 ************************************************/
static void VSYNC_GPIO_Configuration(void)
{
      GPIO_InitTypeDef GPIO_InitStructure;
/* gpio ���ã�����VSYNC���ж�ΪPA4*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);
}



/************************************************
 * ��������VSYNC_NVIC_Configuration
 * ����  ��VSYNC�ж�����
 * ����  ����
 * ���  ����
 * ע��  ����
 ************************************************/
static  void VSYNC_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
/*���ڳ��ж�************************************/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}





/************************************************
 * ��������VSYNC_EXTI_Configuration
 * ����  ��OV7670 VSYNC�жϹܽ�����
 * ����  ����
 * ���  ����
 * ע��  ����
 ************************************************/
static  void VSYNC_EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  
  /* ���ж� */
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&EXTI_InitStructure);
}





/************************************************
 * ��������VSYNC_Init
 * ����  ��OV7670 VSYNC�ж��������
 * ����  ����
 * ���  ����
 * ע��  ����
 ************************************************/
void VSYNC_Init(void)
{
    VSYNC_GPIO_Configuration();
    VSYNC_EXTI_Configuration();
    VSYNC_NVIC_Configuration();
}
/************************************************
 * �������� EXTI4_IRQHandler
 * ����  �����жϴ�����
 * ����  ����
 * ���  ����
 * ע��  ����
 ************************************************/
void EXTI4_IRQHandler(void)
{
    if ( EXTI_GetITStatus(EXTI_Line4) != RESET ) 	//���EXTI_Line4��·�ϵ��ж������Ƿ��͵���NVIC 
    {
        if( OV7670_VSYNC == 0 )
        {
            FIFO_WRST_L(); 	//����ʹFIFOд(����from����ͷ)ָ�븴λ
            FIFO_WE_H();	//����ʹFIFOд����
            OV7670_VSYNC = 1;	   	
            FIFO_WE_H();   //ʹFIFOд����
            FIFO_WRST_H(); //����ʹFIFOд(����from����ͷ)ָ���˶�
        }
        else if( OV7670_VSYNC == 1 )
        {	
			OV7670_VSYNC = 2;
            FIFO_WE_L(); //����ʹFIFOд��ͣ

         } 		
        EXTI_ClearITPendingBit(EXTI_Line4);		//���EXTI_Line4��·�����־λ
    }
  
}

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
      printf("\r\n����:SCCBд���ݴ���\r\n");
        
        Delay_ms(50);
        return ERROR ;
    }
    Delay_ms(50);
    if( 0 == SCCB_ReadByte( &Sensor_IDCode,  0x0b ) )	 /* ��ȡsensor ID��*/
    {     
       printf("\r\n����:��ȡIDʧ��\r\n");
       
		return ERROR;

    }
    printf("\r\nGet ID success��SENSOR ID is 0x%x\r\n",Sensor_IDCode);
    //USART_SendData(USART1, Sensor_IDCode);
   // while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);
    
    
    //DEBUG("Get ID success��SENSOR ID is 0x%x", Sensor_IDCode);
   // DEBUG("Config Register Number is %d ", OV7670_REG_NUM);
    u8 regBuf = 0;
    if(Sensor_IDCode == OV7670_ID)
    {
        for( i = 0 ; i < OV7670_REG_NUM ; i++ )
        {
            if( 0 == SCCB_WriteByte(Sensor_Config[i].Address, Sensor_Config[i].Value) )
            {                
		printf("����:д�Ĵ���0x%xʧ��", Sensor_Config[i].Address);		
              //DEBUG("����:д�Ĵ���0x%xʧ��", Sensor_Config[i].Address);
				return ERROR;
            }
        }
                for( i = 0 ; i < OV7670_REG_NUM ; i++ )
        {
            if( 0 == SCCB_ReadByte(&regBuf, Sensor_Config[i].Address))
            {                
		printf("����:���Ĵ���0x%xʧ��", Sensor_Config[i].Address);		
              //DEBUG("����:д�Ĵ���0x%xʧ��", Sensor_Config[i].Address);
				return ERROR;
            }
            else
            {
              printf("���Ĵ���0x%x�ɹ�,��ֵΪ0x%x", Sensor_Config[i].Address,regBuf);
            }
        }
    }
    else
    {
        return ERROR;
    }
	printf("ov7670 config succeed");
    return SUCCESS;
}


