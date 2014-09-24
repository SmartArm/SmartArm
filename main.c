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

extern void Delay_us(__IO uint32_t nTime);//��ʱus����



int main(void)
{
       NVIC_Config();
       USART1_Config();
       TIM3_PWM_Init();
       TIM2_PWM_Init();
       //��ʼ������1���жϷ�ʽ����

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
         USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//��ʱ�رս����ж�
         data=GetIntData();
         pitch=*(data+3);
         roll=*(data+4);
         yaw=*(data+5);
         pinch=*(data+6);
         
         runServoTo(1,roll);
         runServoTo(2,pitch);
         runServoTo(3,yaw);
         if(pinch<30){
            runServoTo(4,10);//��צ��
         }
         else runServoTo(4,60);//��צ��
        
         ready=0;
         length=0;
         free(data);//�ͷ��ڴ�
        
         USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//�򿪽����ж�
          }
        }
        
        
   
}
/*�ú������������ڵ�����ת��Ϊint���ͣ�Ȼ��int�����ָ�뷵��*/
int *GetIntData()
{
 
  
  int beginPos[dataNum],endPos[dataNum];//��¼ÿ�����ݵĿ�ʼ�ͽ�����λ��
  beginPos[0]=0;
  int num=0;
  for(int x=0;x<length;x++)
  {
    if(USART1_RECV_BUF[x]==' '||USART1_RECV_BUF[x]=='\0')
    {
      endPos[num]=x-1;
      
      num++;
      if(num==dataNum ) break;//��dataNum������
      beginPos[num]=x+1;
    
    }
  }
  num=0;
  char temp[10];//��ʱ��ŷָ�õ��ַ���
  int *Data=(int*)calloc(dataNum,sizeof(int));//���ڴ洢ת���������
  for(int x=0;x<dataNum;x++)
  {

    for(int y=beginPos[x];y<=endPos[x];y++)
    {
      if((USART1_RECV_BUF[y]<48&&USART1_RECV_BUF[y]!=45)||USART1_RECV_BUF[y]>57) continue;//�����Ƿ��ַ�
      temp[num]=USART1_RECV_BUF[y];
      num++;
      if(y==endPos[x])
      {
        temp[num]='\0';// ���һλ������
        num=0;
      }
    }
    (*(Data+x))=atoi(temp); //ת��Ϊ����
    
 
  }
  return Data;

}

void runServoTo(int servo,int degree)//��leapmotion�ĽǶ�����ת��Ϊ���������
{
  
  int position;

  switch (servo){
  case 1: position=degree*(-19)+3010;
          TIM_SetCompare1 (TIM3,position);//����Ķ��
           break;
  case 2: position=degree*(19)+3010;
          TIM_SetCompare2 (TIM3,position);//����ĸ������
           break;
  case 3: position=degree*(-19)+3010;
          TIM_SetCompare3 (TIM3,position);//������ת���
           break;  
  case 4:
           position=degree*(-19)+3010;
          TIM_SetCompare4 (TIM3,position);//צ��
           break;   
  case 5:
           position=degree*(-19)+3010;
          TIM_SetCompare1 (TIM2,position);//λ�ö��1
           break;  
  case 6:
           position=degree*(-19)+3010;
          TIM_SetCompare2 (TIM2,position);//λ�ö��2
           break;           
  default:position=degree*19+3010;
           break;
  }
  

  

  
}

