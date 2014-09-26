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

}//初始化小车控制方向引脚
void advance(){
      GPIO_SetBits(GPIOB,GPIO_Pin_1);
      GPIO_ResetBits(GPIOB,GPIO_Pin_2);//将PB14拉低，PB15拉高
      
      GPIO_SetBits(GPIOC,GPIO_Pin_2);
      GPIO_ResetBits(GPIOC,GPIO_Pin_3);//将PC14 拉低，PC15拉高

}//将小车方向设为前进
void back(){
      GPIO_SetBits(GPIOA,GPIO_Pin_2);
      GPIO_ResetBits(GPIOA,GPIO_Pin_3);//将PB15拉低，PB14拉高
      
      GPIO_SetBits(GPIOC,GPIO_Pin_2);
      GPIO_ResetBits(GPIOC,GPIO_Pin_3);//将PC14 拉低，PC15拉高

}//将小车方向设为后退
void stop(){
      GPIO_ResetBits(GPIOA,GPIO_Pin_2);
      GPIO_ResetBits(GPIOA,GPIO_Pin_3);//将PB15拉低，PB14拉低
      
      GPIO_ResetBits(GPIOC,GPIO_Pin_2);
      GPIO_ResetBits(GPIOC,GPIO_Pin_3);//将PC14 拉低，PC15拉低

}//制动小车
void setLeftSpeed(int speed){
  TIM_SetCompare1 (TIM1, speed);

}//设置左轮速度
void setRightSpeed(int speed){
  TIM_SetCompare2 (TIM1, speed);

}//设置右轮速度
