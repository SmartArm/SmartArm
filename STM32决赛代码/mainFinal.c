//#include"arm.h"
#include"pwm.h"
#include"Timer.h"
#include"wifi.h"
#include<stdio.h>
#include<stdlib.h>
#include<stm32f4xx_it.h>
#include<math.h>
#include"SysTick.h"
#define PI 3.141592653
#define dataNum 12
#define l1 100 //��۳���mm
#define l2 95 //С�۳���mm
char USART1_RECV_BUF[150];
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
  {12,1},{0,2},{0,3},{60,4},{40,5},{0,6}
};
struct Spherical_coor{
  float r;
  int phi;
  int theta;
}S_Coordinate={195,180,0};

struct servo* servo_ptr=&servos[0];  
int theta1=0,theta2=0;//theta1�Ǵ�۶���Ƕ�,theta2��С�۶���Ƕ�
int armStatus=1;//�ֱ�״̬��¼��0�����ڸ�λ״̬��1������������״̬

void runServoToByStep(struct servo* s_ptr,int target,int step,int interval);
void runServoTo(struct servo* s_ptr,int target);
void armInit(void);
//void armReset(void);
#define radius 50;//���ڻ�Բ�����İ뾶
void DrawCircle(int cx,int cy,int cz);//��е���ڵ�ǰˮƽ���ϻ�Բ  
void up_and_down(void);//��е�۵�ͷ
int Mode=1,valid=0,x=0,y=0,z=0,pitch=90,roll=90,yaw=0,pinch=100, circleGesture=0,tapGesture=0,speed=0;//Mode�������ʲô����ģʽ��1�����ֶ����ƣ�2�����Զ�׷��
//valid����ǰ�û������Ƿ���ʶ������
//uint32_t Time1=0,Time2=0,t=0;
void angleCalc(int _x,int _y,int _z,int _pitch);//�������ĽǶ�
void controlServo(int t1,int t2,int t3,int t4,int t5,int t6);//ִ�п�������
/*********************************�������Զ�׷��PID�㷨�ĺ���************************************************/
#define CENTERX 320
#define CENTERY 240
int First=1;
typedef struct PID
{
  int SetPoint; //�趨Ŀ�� Desired Value
  long SumError; //����ۼ�
  double Proportion;//�������� Proportional Const
  double Integral; //���ֳ��� Integral Const
  double Derivative; //΢�ֳ��� Derivative Const
  int LastError; //Error[-1]
  int PrevError; //Error[-2]
};
u8 PIDTime=0;//���ñ���Ϊ1ʱ��˵������ʱ�䵽����Ҫ����PID����
static struct PID xPID={0,0,0,0,0,0,0},yPID={0,0,0,0,0,0,0};//��ʼ������PID�ṹ��
static struct PID *Xptr = &xPID;
static struct PID *Yptr = &yPID;
int outputX=0,outputY=200,outputZ=100;//��ʼ����е�۵��������̬
//����ʽPID�������
int IncPIDCalc(int NextPoint,struct PID * ptr)
{
  register int iError, iIncpid;
  //��ǰ���
  iError = ptr->SetPoint - NextPoint;
  //��������
  iIncpid = ptr->Proportion * iError //E[k]��
          - ptr->Integral * ptr->LastError //E[k��1]��
          + ptr->Derivative * ptr->PrevError; //E[k��2]��
  //�洢�������´μ���
  ptr->PrevError = ptr->LastError;
  ptr->LastError = iError;
  //��������ֵ
  return iIncpid;
}
/************************���ñ�������*************************/
void SetProportion(struct PID * ptr,double p)
{
  ptr->Proportion=p;
}
/************************����΢�ֲ���*************************/
void SetDerivative(struct PID * ptr,double d)
{
  ptr->Derivative=d;
}
/************************����Ŀ��*************************/
void SetTarget(struct PID * ptr,double target)
{
  ptr->SetPoint=target;
}
/*********************************PID�㷨�ĺ�������ν���************************************************/
int main(void)
{
       NVIC_Config();
       USART1_Config();
       TIM3_PWM_Init();
       TIM2_PWM_Init();
       TIM1_GPIO_Config();
       Tim1_Config();
       TIM4_Config();//PID��ʱ
       //TIM_SetCompare1 (TIM1,10000);
       //��ʼ������1���жϷ�ʽ����
       SysTick_Init();//��ʼ��ʱ��
       SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;//ʹ��ʱ��

       armInit();//��ʼ����е��
      // runServoTo(servo_ptr+4,120);//��۶��,������40��ʱ���պô�ֱ����ҪУ�����ƫ����
      //  runServoTo(servo_ptr+4,-60);//��۶��,������40��ʱ���պô�ֱ����ҪУ�����ƫ����
       //runServoTo(servo_ptr+3,60);//��۶��
      // runServoTo(servo_ptr+3,-10);//��۶��
       
       //runServoTo(servo_ptr+3,-90);//��۶��
       SetTarget(Xptr,CENTERX);
       SetTarget(Yptr,CENTERY);//��׷�ٵ�PID�㷨��Ŀ������Ϊ���ĵ�




        while(1){
         //Time1=getTime();
         servo_ptr=&servos[0];
         int *data;
         if(ready==1){//����׼����
        
         USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//��ʱ�رս����ж�
         
         data=GetIntData();

         Mode=*(data);//�ȶ�ȡģʽ
         if(Mode=1)//��������ֶ�����ģʽ
         {
           int First=1;//��PID�㷨�е��״α�־��Ϊ1
           valid=*(data+1);
           x=*(data+2);
           y=*(data+3);
           z=*(data+4);
           //if((z>-90&&z<200)&&(x>-200&&x<200)&&(y>80&&y<400)) {
             
           y=y-50;//������ϵ��Y��ƽ�ƣ��������
           z=z-200;//������ϵ��Z��ƽ�ƣ��������
           if(z>0) z=0;//���Ʊ߽�
           pitch=*(data+5);
           roll=*(data+6);
           roll=roll+12;//����ƫ��
           yaw=*(data+7);
           pinch=*(data+8);
           circleGesture=*(data+9);
           tapGesture=*(data+10);
           speed=*(data+11);//�û��ֵ��ƶ��ٶ�
            
  
           if(valid==0)
           {
             ready=0;
             length=0;
             free(data);//�ͷ��ڴ�
             USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//�򿪽����ж�  
             continue;
           }
             /*********************************/
             /*����������ִ�ж���ʱ�Ŀ��ƴ���*/
             /********************************/
              else
              {
              
              if(speed<10)
              {
                  armStatus=1;//�����ֱ�״̬
                  ready=0;
                  length=0;
                  free(data);//�ͷ��ڴ�
                  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//�򿪽����ж�
                  continue;
              
              }//���ֵ��ٶȹ�С����Ϊ�Ǿ�ֹ��
         
              angleCalc(x,y,z,pitch);//����Ƕ�
              int clawAngle;
              if(pinch<60)
              {
                  clawAngle=-10;
                 //��צ��
                }
              else  clawAngle=60;
     
     
              controlServo(roll,pitch,yaw,clawAngle,theta1,theta2);//ִ�п������� //t1~t6�ֱ��Ӧ����ROLL,����PITCH�������Ƕȣ�צ�ӣ���ۣ�С��
               
             /* if( circleGesture>=2)
              {
                 DrawCircle(x,y,z);  
                
     
                          
              }//��Ȧ��������
              if(tapGesture==1)
              {
                
                 up_and_down();
              }
            */
        
              armStatus=1;//�����ֱ�״̬
              ready=0;
              length=0;
              free(data);//�ͷ��ڴ�
               
              USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//�򿪽����ж�
       
         
         }
         
        }
        else if(Mode==2)
         {
           if(First==1)
           {
            //��һ�ν���,����������������λ��
            angleCalc(outputX,outputY,outputZ,pitch);//����Ƕ�
            controlServo(0,pitch,-S_Coordinate.phi,60,theta1,theta2);//ִ�п������� //t1~t6�ֱ��Ӧ����ROLL,����PITCH�������Ƕȣ�צ�ӣ���ۣ�С��
            Delay_ms(500);
            First=0;
           }
           //�����Զ�׷��ģʽ,ÿ0.1Sִ��һ��PID����
           if(PIDTime==1)
           {
             int Pause,CamX,CamY;//׷���Ƿ���ͣ����׷��������������Ļ��λ��
             Pause=*(data+1);
             CamX=*(data+2);
             CamY=*(data+3);//��ȡ����

             outputX+=IncPIDCalc(CamX,Xptr);
             outputY+=IncPIDCalc(CamY,Yptr);
             
             PIDTime=0;
             TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE); //��TIM4 �ж� 
           }
         }
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



void armInit(){
    USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//��ʱ�رս����ж�
  
       for(int x=0;x<6;x++)
       {
         int target=(*servo_ptr).lastPos;
         runServoTo(servo_ptr,target);
         servo_ptr++;
       }
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//�򿪽����ж�

}
//****************************�����Ǽ���λ�õĺ���*****************************/
//����Ƕȵĸ�������
struct Spherical_coor change_coordinate(int x,int y,int z){
  struct Spherical_coor temp;
  temp.r=sqrt(x*x+y*y+z*z);
  
  double phi_temp,theta_temp;
  phi_temp=atan((double)x/(double)z)*180/PI;
  theta_temp=acos(y/temp.r)*180/PI;
  temp.phi=(int) phi_temp;
  temp.theta=(int) theta_temp;
  //temp.theta=-temp.theta;   //��phi?
  if(temp.r>190) temp.r=190;//���Ƴ���
  return temp;
}
void setTheta(int x,int y,int z){
 double r=S_Coordinate.r;
 double angleTemp1=0,angleTemp2=0;
 angleTemp1=acos((r*r+975)/(200*r))*180/PI;
 angleTemp2=acos((19025-r*r)/19000)*180/PI;
 theta1=(int) S_Coordinate.theta-angleTemp1;
// if(z>-20) theta1=-theta1;
 theta1=-theta1;
 theta2=180-(int) angleTemp2;
}
//***********************************����X,Y,Z�������۶����С�۶���ĽǶ�*************************/
void angleCalc(int _x,int _y,int _z,int _pitch)
{
   S_Coordinate=change_coordinate(_x,_y,_z);
   setTheta(_x,_y,_z);
   pitch=_pitch+theta2;//���С�۶���ĽǶȵ��������ǣ�ʹ�þ���ƽ��ˮƽ��
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
//t1~t6�ֱ��Ӧ����ROLL,����PITCH�������Ƕȣ�צ�ӣ���ۣ�С��
void controlServo(int t1,int t2,int t3,int t4,int t5,int t6)
{
   t5=t5+40;//������۶����ƫ����
   if(t2>90) t2=90;
   else if(t2<-90) t2=-90;
   if(t5>120) t5=120;
   else if(t5<-60) t5=-60;
   if(t6>90) t6=90;
   else if(t6<-90) t6=-90;//���ƽǶ�

   runServoTo(servo_ptr,t1);
   runServoTo(servo_ptr+1,t2);
   if(S_Coordinate.r>100)
    {
           runServoTo(servo_ptr+2, -S_Coordinate.phi);//�������         
    }
    else
    {
           runServoTo(servo_ptr+2,yaw);//�������      
    }
   runServoTo(servo_ptr+2,t3);//�������    
   runServoTo(servo_ptr+3,t4);//��צ��
   runServoTo(servo_ptr+4,t5);//��۶��
   runServoTo(servo_ptr+5,t6);//С�۶��      
}
/************************************************************/
//������:runServoToByStep
//���룺����ṹ��ָ�룬Ŀ��Ƕȣ������Ƕȣ�ÿ��ʱ������ms��
//����ֵ����
//�����������������ĳһ��������ض�������ʱ��������ĳһ�Ƕ�
/************************************************************/

void runServoToByStep(struct servo* s_ptr,int target,int step,int interval)
{
  
  int pwm,position=(*s_ptr).lastPos;
  if(position-target==0) return;
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
           pwm=position*(19)+3010;
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
           TIM_SetCompare4(TIM3,pwm);//צ��
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
void armReset(void)
{
           runServoToByStep(servo_ptr,0,1,8);
           runServoToByStep(servo_ptr+1,0,1,8);
           runServoToByStep(servo_ptr+2,0,1,8);
           runServoToByStep(servo_ptr+3,60,1,8);
           runServoToByStep(servo_ptr+5,45,1,8);
           runServoToByStep(servo_ptr+4,90,1,8);
           runServoToByStep(servo_ptr+5,90,1,8);
}
void Nod_Nod(void)//��е�۵�ͷ
{
  int memory[6];
  for(int x=0;x<6;x++)
  {
    memory[x]=servos[x].lastPos;
  }//��¼����ǰ��λ��
    runServoToByStep(servo_ptr,0,1,8);
    runServoToByStep(servo_ptr+2,0,1,8);
    runServoToByStep(servo_ptr+3,60,1,8);
    runServoToByStep(servo_ptr+4,0,1,8);
    runServoToByStep(servo_ptr+5,0,1,8);
    
     runServoToByStep(servo_ptr+1,0,1,8);
     runServoToByStep(servo_ptr+1,-45,1,8);
     runServoToByStep(servo_ptr+1,0,1,8);
     runServoToByStep(servo_ptr+1,45,1,8);
     runServoToByStep(servo_ptr+1,0,1,8);
     
    /* for(int y=6;y>0;y--)
     {
       runServoToByStep(servo_ptr+y,memory[y],1,6);
     
     }*///�ָ�������ǰ

     
}
void DrawCircle(int cx,int cy,int cz)//��е���ڵ�ǰˮƽ���ϻ�Բ  
{
  double angle;//0.017453Լ����1��
  int X,Y,Z;
  for(int i=0;i<5;i++)
  {
  for(angle=0;angle<=2*PI;angle+=0.087)//5��Ϊ����
  {
    X=(int) cx+cos(angle)*radius;
    Z=(int) cz+sin(angle)*radius;
    Y=cy;
    S_Coordinate=change_coordinate(X,Y,Z);
    setTheta(X,Y,Z);
    
    runServoTo(servo_ptr,0);
         
    pitch=pitch+theta2;//���С�۶���ĽǶȵ��������ǣ�ʹ�þ���ƽ��ˮƽ��
    if(pitch>90) pitch=90;
    if(pitch<-90) pitch=-90;
    runServoTo(servo_ptr+1,pitch);


    runServoTo(servo_ptr+3,-10);//��צ��


    runServoTo(servo_ptr+2, -S_Coordinate.phi);//�������         


    if(theta2>90) theta2=90;
    else if(theta2<-90) theta2=-90;
    if(theta1>90) theta1=90;
    else if(theta1<-90) theta1=-90;

    runServoTo(servo_ptr+4,theta1);//��۶��
    runServoTo(servo_ptr+5,theta2);//С�۶�� 
    
    Delay_ms(40);
    
  }
  }
}
void up_and_down(void)
{
    int i,j,k;
    runServoToByStep(servo_ptr,12,1,5);
    runServoToByStep(servo_ptr+2,0,1,5);
    runServoToByStep(servo_ptr+1,22,1,5);
    runServoToByStep(servo_ptr+4,66,1,5);
    runServoToByStep(servo_ptr+5,89,1,5);
    runServoToByStep(servo_ptr+3,60,1,5);
    Delay_ms(500);
    for(i=0;i<3;i++)
    {
      //runServoToByStep(servo_ptr+3,-10,1,5);
      for(j=1;j<=45;j++)
      {
        runServoToByStep(servo_ptr+1,22-j,1,1);
        runServoToByStep(servo_ptr+4,66-j,1,1);
        runServoToByStep(servo_ptr+5,89-2*j,1,1);
        Delay_ms(20);
      }
      //runServoTo(servo_ptr+1,-25);
      //Delay_ms(1000);
      runServoToByStep(servo_ptr+3,60,1,5);
      Delay_ms(500);
      runServoToByStep(servo_ptr+3,-10,1,5);
      for(k=1;k<=45;k++)
      {
        runServoToByStep(servo_ptr+1,-23+k,1,1);
        runServoToByStep(servo_ptr+4,21+k,1,1);
        runServoToByStep(servo_ptr+5,-1+2*k,1,1);
        Delay_ms(20);
      }
     //runServoTo(servo_ptr+1,20);
     // Delay_ms(1000);
      runServoToByStep(servo_ptr+3,60,1,5);
      Delay_ms(500);
    }
}

