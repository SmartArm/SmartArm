#include"DMA.h"

/*
 * 函数名：DMA_Config
 * 描述  ：DMA 串口的初始化配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_DMA2, ENABLE);	//开启DMA时钟
    DMA_NVIC_Config();	   			//配置DMA中断
    DMA_DeInit(DMA2_Stream7);
    DMA_StructInit( &DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
 	/*设置DMA源：内存地址&串口数据寄存器地址*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);	   

	/*内存地址(要传输的变量的指针)*/
    DMA_InitStructure.DMA_Memory0BaseAddr  = (uint32_t) buffer;
	
	/*方向：从内存到外设*/		
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;	
	
	/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
    DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;
	
	/*外设地址不增*/	    
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	
	/*内存地址自增*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	
	/*外设数据单位*/	
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	/*内存数据单位 8bit*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
	
	/*DMA模式：一次传输，循环*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;	
      DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	
	/*优先级：中*/	
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
	

	
	/*配置DMA1的4通道*/		   
    DMA_Init(DMA2_Stream7, &DMA_InitStructure); 	   
	
	DMA_Cmd (DMA2_Stream7,ENABLE);					//使能DMA
        
      
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	
}
/*
 * 函数名：NVIC_Config
 * 描述  ：DMA 中断配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
static void DMA_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* 配置P[A|B|C|D|E]0为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/*
 * 函数名：DMA1_Channel4_IRQHandler
 * 描述  ：DMA 中断处理函数
 * 输入  ：无
 * 输出  : 无
 * 调用  ：
 */
void DMA2_Stream7_IRQHandler(void)
{	
//判断是否为DMA发送完成中断
   if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)==SET) 
   {  
        
        DMAstate++;//传输完成	
        DMAComplete=1;	
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7); 
        
        
    }
   if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TEIF7)==SET)
   {
     DMAstate=999;
     
   }
   
}
