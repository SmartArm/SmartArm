#include"pwm.h"
#include"wifi.h"
#include<stdio.h>
#include<stdlib.h>
#include<stm32f4xx_it.h>
#include<math.h>
#define dataNum 7
char USART1_RECV_BUF[100];
int ready=0;
int length=0;
int *GetIntData();
void runServoTo(int servo,int degree);

extern void Delay_us(__IO uint32_t nTime);//延时us函数



int main(void)
{
       NVIC_Config();
       USART1_Config();
       TIM3_PWM_Init();
       TIM2_PWM_Init();
       //初始化串口1，中断方式接收

       int x=0,y=0,z=0,pitch=90,roll=90,yaw=0,pinch=100;
       int instruction[7]={0,0,0,90,90,0,100};// x=0,y=0,z=0,pitch=90,roll=90,yaw=0,pinch=100;
       runServoTo(4,60);
       runServoTo(5,0);
         runServoTo(6,0);
      /* for(int x=0;x<7;x++)
       {
          runServoTo(x+1,instruction[x]);
 
       }*/


        while(1){
   

         int *data;
         if(ready==1){
         USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//暂时关闭接收中断
         data=GetIntData();
         pitch=*(data+3);
         roll=*(data+4);
         yaw=*(data+5);
         pinch=*(data+6);
         
         runServoTo(1,roll);
         runServoTo(2,pitch);
         runServoTo(3,yaw);
         if(pinch<30){
            runServoTo(4,10);//合爪子
         }
         else runServoTo(4,60);//张爪子
        
         ready=0;
         length=0;
         free(data);//释放内存
        
         USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//打开接收中断
          }
        }
        
        
   
}
/*该函数将缓冲区内的数据转化为int类型，然后将int数组的指针返回*/
int *GetIntData()
{
 
  
  int beginPos[dataNum],endPos[dataNum];//记录每个数据的开始和结束的位置
  beginPos[0]=0;
  int num=0;
  for(int x=0;x<length;x++)
  {
    if(USART1_RECV_BUF[x]==' '||USART1_RECV_BUF[x]=='\0')
    {
      endPos[num]=x-1;
      
      num++;
      if(num==dataNum ) break;//有dataNum个数据
      beginPos[num]=x+1;
    
    }
  }
  num=0;
  char temp[10];//临时存放分割好的字符串
  int *Data=(int*)calloc(dataNum,sizeof(int));//用于存储转化后的数据
  for(int x=0;x<dataNum;x++)
  {

    for(int y=beginPos[x];y<=endPos[x];y++)
    {
      if((USART1_RECV_BUF[y]<48&&USART1_RECV_BUF[y]!=45)||USART1_RECV_BUF[y]>57) continue;//跳过非法字符
      temp[num]=USART1_RECV_BUF[y];
      num++;
      if(y==endPos[x])
      {
        temp[num]='\0';// 最后一位结束符
        num=0;
      }
    }
    (*(Data+x))=atoi(temp); //转化为整数
    
 
  }
  return Data;

}

void runServoTo(int servo,int degree)//将leapmotion的角度数据转化为舵机的命令
{
  
  int position;

  switch (servo){
  case 1: position=degree*(-19)+3010;
          TIM_SetCompare1 (TIM3,position);//手腕的舵机
           break;
  case 2: position=degree*(19)+3010;
          TIM_SetCompare2 (TIM3,position);//手腕的俯仰舵机
           break;
  case 3: position=degree*(-19)+3010;
          TIM_SetCompare3 (TIM3,position);//底座旋转舵机
           break;  
  case 4:
           position=degree*(-19)+3010;
          TIM_SetCompare4 (TIM3,position);//爪子
           break;   
  case 5:
           position=degree*(-19)+3010;
          TIM_SetCompare1 (TIM2,position);//位置舵机1
           break;  
  case 6:
           position=degree*(-19)+3010;
          TIM_SetCompare2 (TIM2,position);//位置舵机2
           break;           
  default:position=degree*19+3010;
           break;
  }
  

  

  
}

