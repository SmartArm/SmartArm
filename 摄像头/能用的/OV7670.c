
#include "SCCB.h"
#include "OV7670.h"
#include "systick.h"
#include "misc.h"
#include "SensorConfig.h"
#include "usart1.h"


#define OV7670_REG_NUM 166
u8 volatile OV7670_VSYNC;	 /* 帧同步信号 */
volatile u8 Frame_Count = 0 ;

extern Register_Info Sensor_Config[];



/************************************************
 * 函数名：FIFO_GPIO_Configuration
 * 描述  ：FIFO GPIO配置
 * 输入  ：无
 * 输出  ：无
 * 注意  ：无
 ************************************************/
void FIFO_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE); 

    /*PC2(FIFO_WRST--FIFO写复位)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*PC3(FIFO_WEN--FIFO写使能)  (FIFO_REN--FIFO读使能直接接地)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*PC4(FIFO_RRST--FIFO读复位)  PC5(FIFO_RCLK-FIFO读时钟)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*PB0-PB7(FIFO_DATA--FIFO输出数据)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//默认即是浮空输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //FIFO_CS_L();	  					/*拉低使FIFO输出使能*//这里硬件上接地了
    FIFO_WE_H();   						/*拉高使FIFO写允许*/

}




/************************************************
 * 函数名：VSYNC_GPIO_Configuration
 * 描述  ：OV7670 GPIO配置
 * 输入  ：无
 * 输出  ：无
 * 注意  ：无
 ************************************************/
static void VSYNC_GPIO_Configuration(void)
{
      GPIO_InitTypeDef GPIO_InitStructure;
/* gpio 配置，配置VSYNC场中断为PA4*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource4);
}



/************************************************
 * 函数名：VSYNC_NVIC_Configuration
 * 描述  ：VSYNC中断配置
 * 输入  ：无
 * 输出  ：无
 * 注意  ：无
 ************************************************/
static  void VSYNC_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
/*用于场中断************************************/
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}





/************************************************
 * 函数名：VSYNC_EXTI_Configuration
 * 描述  ：OV7670 VSYNC中断管脚配置
 * 输入  ：无
 * 输出  ：无
 * 注意  ：无
 ************************************************/
static  void VSYNC_EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  
  /* 场中断 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&EXTI_InitStructure);
}





/************************************************
 * 函数名：VSYNC_Init
 * 描述  ：OV7670 VSYNC中断相关配置
 * 输入  ：无
 * 输出  ：无
 * 注意  ：无
 ************************************************/
void VSYNC_Init(void)
{
    VSYNC_GPIO_Configuration();
    VSYNC_EXTI_Configuration();
    VSYNC_NVIC_Configuration();
}
/************************************************
 * 函数名： EXTI4_IRQHandler
 * 描述  ：场中断处理函数
 * 输入  ：无
 * 输出  ：无
 * 注意  ：无
 ************************************************/
void EXTI4_IRQHandler(void)
{
    if ( EXTI_GetITStatus(EXTI_Line4) != RESET ) 	//检查EXTI_Line4线路上的中断请求是否发送到了NVIC 
    {
        if( OV7670_VSYNC == 0 )
        {
            FIFO_WRST_L(); 	//拉低使FIFO写(数据from摄像头)指针复位
            FIFO_WE_H();	//拉高使FIFO写允许
            OV7670_VSYNC = 1;	   	
            FIFO_WE_H();   //使FIFO写允许
            FIFO_WRST_H(); //允许使FIFO写(数据from摄像头)指针运动
        }
        else if( OV7670_VSYNC == 1 )
        {	
			OV7670_VSYNC = 2;
            FIFO_WE_L(); //拉低使FIFO写暂停

         } 		
        EXTI_ClearITPendingBit(EXTI_Line4);		//清除EXTI_Line4线路挂起标志位
    }
  
}

/************************************************
 * 函数名：Sensor_Init
 * 描述  ：Sensor初始化
 * 输入  ：无
 * 输出  ：返回1成功，返回0失败
 * 注意  ：无
 ************************************************/
ErrorStatus OV7670_Init(void)
{
    u16 i = 0;
    u8 Sensor_IDCode = 0;
	printf("\r\n>>>>>>STM32F103 OV7670&FIFO Demo<<<<<<\r\n");
	//DEBUG("OV7670 Register Config Start!");
    if( 0 == SCCB_WriteByte ( 0x12, 0x80 ) ) /*复位sensor */
    {
      printf("\r\n警告:SCCB写数据错误\r\n");
        
        Delay_ms(50);
        return ERROR ;
    }
    Delay_ms(50);
    if( 0 == SCCB_ReadByte( &Sensor_IDCode,  0x0b ) )	 /* 读取sensor ID号*/
    {     
       printf("\r\n警告:读取ID失败\r\n");
       
		return ERROR;

    }
    printf("\r\nGet ID success，SENSOR ID is 0x%x\r\n",Sensor_IDCode);
    //USART_SendData(USART1, Sensor_IDCode);
   // while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);
    
    
    //DEBUG("Get ID success，SENSOR ID is 0x%x", Sensor_IDCode);
   // DEBUG("Config Register Number is %d ", OV7670_REG_NUM);
    u8 regBuf = 0;
    if(Sensor_IDCode == OV7670_ID)
    {
        for( i = 0 ; i < OV7670_REG_NUM ; i++ )
        {
            if( 0 == SCCB_WriteByte(Sensor_Config[i].Address, Sensor_Config[i].Value) )
            {                
		printf("警告:写寄存器0x%x失败", Sensor_Config[i].Address);		
              //DEBUG("警告:写寄存器0x%x失败", Sensor_Config[i].Address);
				return ERROR;
            }
        }
                for( i = 0 ; i < OV7670_REG_NUM ; i++ )
        {
            if( 0 == SCCB_ReadByte(&regBuf, Sensor_Config[i].Address))
            {                
		printf("警告:读寄存器0x%x失败", Sensor_Config[i].Address);		
              //DEBUG("警告:写寄存器0x%x失败", Sensor_Config[i].Address);
				return ERROR;
            }
            else
            {
              printf("读寄存器0x%x成功,数值为0x%x", Sensor_Config[i].Address,regBuf);
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


