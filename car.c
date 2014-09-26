#include"car.h"
void directionInit(){

      GPIO_InitTypeDef  GPIO_InitStructure;

      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 |GPIO_Pin_3 ;

      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
      

      GPIO_Init(GPIOC, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 |GPIO_Pin_2 ;
           GPIO_Init(GPIOB, &GPIO_InitStructure);

}//��ʼ��С�����Ʒ�������
void advance(){
      GPIO_SetBits(GPIOB,GPIO_Pin_1);
      GPIO_ResetBits(GPIOB,GPIO_Pin_2);//��PB14���ͣ�PB15����
      
      GPIO_SetBits(GPIOC,GPIO_Pin_2);
      GPIO_ResetBits(GPIOC,GPIO_Pin_3);//��PC14 ���ͣ�PC15����

}//��С��������Ϊǰ��
void back(){
      GPIO_SetBits(GPIOA,GPIO_Pin_2);
      GPIO_ResetBits(GPIOA,GPIO_Pin_3);//��PB15���ͣ�PB14����
      
      GPIO_SetBits(GPIOC,GPIO_Pin_2);
      GPIO_ResetBits(GPIOC,GPIO_Pin_3);//��PC14 ���ͣ�PC15����

}//��С��������Ϊ����
void stop(){
      GPIO_ResetBits(GPIOA,GPIO_Pin_2);
      GPIO_ResetBits(GPIOA,GPIO_Pin_3);//��PB15���ͣ�PB14����
      
      GPIO_ResetBits(GPIOC,GPIO_Pin_2);
      GPIO_ResetBits(GPIOC,GPIO_Pin_3);//��PC14 ���ͣ�PC15����

}//�ƶ�С��
void setLeftSpeed(int speed){
  TIM_SetCompare1 (TIM1, speed);

}//���������ٶ�
void setRightSpeed(int speed){
  TIM_SetCompare2 (TIM1, speed);

}//���������ٶ�
