#include"DMA.h"

/*
 * ��������DMA_Config
 * ����  ��DMA ���ڵĳ�ʼ������
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_DMA2, ENABLE);	//����DMAʱ��
    DMA_NVIC_Config();	   			//����DMA�ж�
    DMA_DeInit(DMA2_Stream7);
    DMA_StructInit( &DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
 	/*����DMAԴ���ڴ��ַ&�������ݼĴ�����ַ*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);	   

	/*�ڴ��ַ(Ҫ����ı�����ָ��)*/
    DMA_InitStructure.DMA_Memory0BaseAddr  = (uint32_t) buffer;
	
	/*���򣺴��ڴ浽����*/		
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;	
	
	/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
    DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;
	
	/*�����ַ����*/	    
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	
	/*�ڴ��ַ����*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	
	/*�������ݵ�λ*/	
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	/*�ڴ����ݵ�λ 8bit*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
	
	/*DMAģʽ��һ�δ��䣬ѭ��*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;	
      DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	
	/*���ȼ�����*/	
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
	

	
	/*����DMA1��4ͨ��*/		   
    DMA_Init(DMA2_Stream7, &DMA_InitStructure); 	   
	
	DMA_Cmd (DMA2_Stream7,ENABLE);					//ʹ��DMA
        
      
	DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	
}
/*
 * ��������NVIC_Config
 * ����  ��DMA �ж�����
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
static void DMA_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* ����P[A|B|C|D|E]0Ϊ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/*
 * ��������DMA1_Channel4_IRQHandler
 * ����  ��DMA �жϴ�����
 * ����  ����
 * ���  : ��
 * ����  ��
 */
void DMA2_Stream7_IRQHandler(void)
{	
//�ж��Ƿ�ΪDMA��������ж�
   if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)==SET) 
   {  
        
        DMAstate++;//�������	
        DMAComplete=1;	
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7); 
        
        
    }
   if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TEIF7)==SET)
   {
     DMAstate=999;
     
   }
   
}
