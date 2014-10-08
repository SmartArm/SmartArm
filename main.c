#include"pwm.h"
#include"wifi.h"
#include<stdio.h>
#include<stdlib.h>
#include<stm32f4xx_it.h>
#include<math.h>
#include"SysTick.h"
#define PI 3.141592653
#define dataNum 8
#define l1 100 //��۳���mm
#define l2 95 //С�۳���mm
char USART1_RECV_BUF[100];
int ready=0;
int length=0;
int *GetIntData();

extern void Delay_ms(__IO uint32_t nTime);//��ʱus����
struct Spherical_coor change_coordinate(int x,int y,int z);
void setTheta(int x,int y,int z);
struct servo{
  int lastPos;
  int label;
}servos[6]={
  {0,1},{0,2},{0,3},{60,4},{0,5},{0,6}
};
struct Spherical_coor{
  float r;
  int phi;
  int theta;
}S_Coordinate={195,180,0};
/************************************************************/
//������:runServoToByStep
//���룺����ṹ��ָ�룬Ŀ��Ƕȣ������Ƕȣ�ÿ��ʱ������ms��
//����ֵ����
//�����������������ĳһ��������ض�������ʱ��������ĳһ�Ƕ�
/************************************************************/

void runServoToByStep(struct servo* s_ptr,int target,int step,int interval)
{
  
  int pwm,position=(*s_ptr).lastPos;

  if(position-target>0) step=-step;
  switch ((*s_ptr).label){
  case 1:if(step>0)
         {
           while(position<target)
          { 
           position+=step;
           pwm=position*(-19)+3010;
           TIM_SetCompare1 (TIM3,pwm);
           Delay_ms(interval);
          }
         }
         else{
           while(position>target)
          { 
           position+=step;
           pwm=position*(-19)+3010;
           TIM_SetCompare1 (TIM3,pwm);
           Delay_ms(interval);
          }
         }
           break;//����Ķ��
  case 2:if(step>0)
         {
           while(position<target)
          { 
           position+=step;
           pwm=position*(-19)+3010;
           TIM_SetCompare2 (TIM3,pwm);
           Delay_ms(interval);
          }
         }
         else{
           while(position>target)
          { 
           position+=step;
           pwm=position*(19)+3010;
           TIM_SetCompare2(TIM3,pwm);
           Delay_ms(interval);
          }
         } 

           break;//����ĸ������
  case 3:if(step>0)
         {
           while(position<target)
          { 
           position+=step;
           pwm=position*(-19)+3010;
           TIM_SetCompare3 (TIM3,pwm);
           Delay_ms(interval);
          }
         }
         else{
           while(position>target)
          { 
           position+=step;
           pwm=position*(-19)+3010;
           TIM_SetCompare3(TIM3,pwm);
           Delay_ms(interval);
          }
         }  
           break;  //������ת���
  case 4:if(step>0)
         {
           while(position<target)
          { 
           position+=step;
           pwm=position*(-19)+3010;
           TIM_SetCompare4 (TIM3,pwm);
           Delay_ms(interval);
          }
         }
         else{
           while(position>target)
          { 
           position+=step;
           pwm=position*(-19)+3010;
           TIM_SetCompare4(TIM3,pwm);
           Delay_ms(interval);
          }
         }  

           break;   //צ��
  case 5:if(step>0)
         {
           while(position<target)
          { 
           position+=step;
           pwm=position*(-19)+3010;
           TIM_SetCompare1 (TIM2,pwm);
           Delay_ms(interval);
          }
         }
         else{
           while(position>target)
          { 
           position+=step;
           pwm=position*(-19)+3010;
           TIM_SetCompare1(TIM2,pwm);
           Delay_ms(interval);
          }
         }  //λ�ö��1

           break;  
  case 6:if(step>0)
         {
           while(position<target)
          { 
           position+=step;
           pwm=position*(-19)+3010;
           TIM_SetCompare2 (TIM2,pwm);
           Delay_ms(interval);
          }
         }
         else{
           while(position>target)
          { 
           position+=step;
           pwm=position*(-19)+3010;
           TIM_SetCompare2(TIM2,pwm);
           Delay_ms(interval);
          }
         }
                  //λ�ö��2
           break;           

  }
  
  (*s_ptr).lastPos=target;//����λ��
  

  
}
void runServoTo(struct servo* s_ptr,int target)
{
  int pwm;

  switch ((*s_ptr).label){
  case 1:
          pwm=target*(-19)+3010;
          TIM_SetCompare1 (TIM3,pwm);
        
           break;//����Ķ��
  case 2: pwm=target*(19)+3010;
          TIM_SetCompare2 (TIM3,pwm);
           break;//����ĸ������
  case 3: pwm=target*(-19)+3010;
          TIM_SetCompare3 (TIM3,pwm);
           break;  //������ת���
  case 4:
           pwm=target*(-19)+3010;
          TIM_SetCompare4 (TIM3,pwm);//צ��
           break;   
  case 5://target+=20;//����ƫ����
           pwm=target*(-19)+3010;
          TIM_SetCompare1 (TIM2,pwm);//λ�ö��1
           break;  
  case 6:
           pwm=target*(-19)+3010;
          TIM_SetCompare2 (TIM2,pwm);//λ�ö��2
           break;           
  default:pwm=target*19+3010;
           break;
  }
  
  (*s_ptr).lastPos=target;//����λ��
  

  
}
struct servo* servo_ptr=&servos[0];      

void armInit(){
       for(int x=0;x<6;x++)
       {
         int target=(*servo_ptr).lastPos;
         runServoTo(servo_ptr,target);
         servo_ptr++;
       }


}
int theta1=0,theta2=0;//theta1�Ǵ�۶���Ƕ�,theta2��С�۶���Ƕ�
double theta3=0,theta4=0;
int main(void)
{
       NVIC_Config();
       USART1_Config();
       TIM3_PWM_Init();
       TIM2_PWM_Init();
       TIM1_GPIO_Config();
       Tim1_Config();

       //��ʼ������1���жϷ�ʽ����
       SysTick_Init();//��ʼ��ʱ��
       SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;//ʹ��ʱ��

       int x=0,y=0,z=0,pitch=90,roll=90,yaw=0,pinch=100, circleGesture=0;

       armInit();//��ʼ����е��

       uint32_t lastTime=0;
       TIM_SetCompare1(TIM1,1000);
       TIM_SetCompare2(TIM1,1000);
       

        while(1){


         servo_ptr=&servos[0];
         int *data;
         if(ready==1){
         USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//��ʱ�رս����ж�
         data=GetIntData();
         x=*data;
         y=*(data+1);
         z=*(data+2);
         y=y-20;//������ϵ��Y��ƽ�ƣ��������
         z=z-60;//������ϵ��Z��ƽ�ƣ��������
         pitch=*(data+3);

         roll=*(data+4);
         yaw=*(data+5);
         pinch=*(data+6);
         circleGesture=*(data+7);
         S_Coordinate=change_coordinate(x,y,z);
         setTheta(x,y,z);
         


         if( circleGesture>=2)
         {
              
           runServoToByStep(servo_ptr,0,1,10);
           runServoToByStep(servo_ptr+1,0,1,10);
           runServoToByStep(servo_ptr+2,0,1,10);
           runServoToByStep(servo_ptr+3,60,1,10);
           runServoToByStep(servo_ptr+5,45,1,10);
           runServoToByStep(servo_ptr+4,90,1,10);
           runServoToByStep(servo_ptr+5,90,1,10);
           Delay_ms(2000);
           
           runServoToByStep(servo_ptr+4,0,1,10);
           runServoToByStep(servo_ptr+5,0,1,10);      
                  
         }//��������
         
         runServoTo(servo_ptr,roll);
         
         pitch=pitch+theta2-45;//���С�۶���ĽǶȵ��������ǣ�ʹ�þ���ƽ��ˮƽ��
         if(pitch>90) pitch=90;
         runServoTo(servo_ptr+1,pitch);
         
         
         //runServoTo(servo_ptr+2,yaw);
         
         runServoTo(servo_ptr+2, -S_Coordinate.phi);//�������
         if(theta2>90) theta2=90;
         else if(theta2<-90) theta2=-90;
         if(theta1>180) theta1=180;
         else if(theta1<0) theta1=0;
   
        
         runServoTo(servo_ptr+4,theta1-90);//��۶��
         runServoTo(servo_ptr+5,theta2);//С�۶��
         uint32_t RunTime=getTime();
         if(RunTime-lastTime>200)//��ֹ���������Ϣʹצ�Ӵ����ź�
         {
           if(pinch<40)
           {
            runServoTo(servo_ptr+3,-10);//��צ��
           }
           else runServoTo(servo_ptr+3,60);//��צ��
           lastTime=RunTime;
         }
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


struct Spherical_coor change_coordinate(int x,int y,int z){
  struct Spherical_coor temp;
  temp.r=sqrt(x*x+y*y+z*z);
  
  double phi_temp,theta_temp;
  phi_temp=atan((double)x/(double)z)*180/PI;
  theta_temp=acos(y/temp.r)*180/PI;
  temp.phi=(int) phi_temp;
  temp.theta=(int) theta_temp;
  temp.theta=-temp.theta;
  if(temp.r>190) temp.r=190;//���Ƴ���
  return temp;
}
void setTheta(int x,int y,int z){
 double r=S_Coordinate.r;
 
 theta3=acos((r*r+975)/(200*r))*180/PI;
 theta4=acos((19025-r*r)/19000)*180/PI;
 theta1=(int) theta3+S_Coordinate.theta+90;
 theta2=180-(int) theta4;
}
