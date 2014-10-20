#include"arm.h"
#include"pwm.h"
#include"wifi.h"
#include<stdio.h>
#include<stdlib.h>
#include<stm32f4xx_it.h>
#include<math.h>
#include"SysTick.h"
#define PI 3.141592653
#define dataNum 10
#define l1 100 //大臂长度mm
#define l2 95 //小臂长度mm
char USART1_RECV_BUF[150];
int ready=0;
int length=0;
int *GetIntData();

extern void Delay_ms(__IO uint32_t nTime);//延时us函数
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

struct servo* servo_ptr=&servos[0];  
int theta1=0,theta2=0;//theta1是大臂舵机角度,theta2是小臂舵机角度
int armStatus=1;//手臂状态记录，0代表处于复位状态，1代表正常动作状态
double theta3=0,theta4=0;
void runServoToByStep(struct servo* s_ptr,int target,int step,int interval);
void runServoTo(struct servo* s_ptr,int target);
void armInit(void);
void armReset(void);
#define radius 50;//用于画圆函数的半径
void DrawCircle(int cx,int cy,int cz);//机械臂在当前水平面上画圆  
void Nod_Nod(void);//机械臂点头
   int valid=0,x=0,y=0,z=0,pitch=90,roll=90,yaw=0,pinch=100, circleGesture=0,tapGesture=0;//valid代表当前用户的手是否在识别区里
int main(void)
{
       NVIC_Config();
       USART1_Config();
       TIM3_PWM_Init();
       TIM2_PWM_Init();
       TIM1_GPIO_Config();
       Tim1_Config();

       //初始化串口1，中断方式接收
       SysTick_Init();//初始化时钟
       SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;//使能时钟

      
     


       armInit();//初始化机械臂

       //uint32_t lastTime=0;
       

        while(1){
          
         servo_ptr=&servos[0];
         int *data;
         if(ready==1){
         USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//暂时关闭接收中断
         
         data=GetIntData();

         valid=*(data);
         x=*(data+1);
         y=*(data+2);
         z=*(data+3);
         //if((z>-90&&z<200)&&(x>-200&&x<200)&&(y>80&&y<400)) {
           
         y=y-50;//将坐标系沿Y轴平移，方便控制
         z=z-200;//将坐标系沿Z轴平移，方便控制
         pitch=*(data+4);
         roll=*(data+5);
         yaw=*(data+6);
         pinch=*(data+7);
         circleGesture=*(data+8);
         tapGesture=*(data+9);
         
       
         if((valid==0)&&(armStatus==1))
         {
           armReset();
           //Delay_ms(2000);
           armStatus=0;//更新手臂状态
           ready=0;
           length=0;
           free(data);//释放内存
           USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//打开接收中断
           //break;
         }
         else if((valid==0)&&(armStatus==0))
         {
           armStatus=0;//更新手臂状态
           ready=0;
           length=0;
           free(data);//释放内存
           USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//打开接收中断
           //break;
         }
         else if((valid==1)&&(armStatus==0))//手再次进入
         {
           S_Coordinate=change_coordinate(x,y,z);
           setTheta(x,y,z);
       
           runServoTo(servo_ptr,roll);
           
           pitch=pitch+theta2;//结合小臂舵机的角度调整俯仰角，使得尽量平行水平面
           if(pitch>90) pitch=90;
           if(pitch<-90) pitch=-90;
           runServoTo(servo_ptr+1,pitch);
           uint32_t RunTime=getTime();

             if(pinch<60)
             {
              runServoTo(servo_ptr+3,-10);//合爪子
             }
             else runServoTo(servo_ptr+3,60);//张爪子

           
           
           //runServoTo(servo_ptr+2,yaw);
           if(S_Coordinate.r>100)
           {
           runServoToByStep(servo_ptr+2, -S_Coordinate.phi,1,5);//底座舵机         
           }
           else
           {
             runServoTo(servo_ptr+2,yaw);//底座舵机      
             
           }
           if(theta2>90) theta2=90;
           else if(theta2<-90) theta2=-90;
           if(theta1>90) theta1=90;
           else if(theta1<-90) theta1=-90;
            runServoToByStep(servo_ptr+4,theta1,1,9);//大臂舵机
            runServoToByStep(servo_ptr+5,theta2,1,9);//小臂舵机
                      

           armStatus=1;//更新手臂状态
           ready=0;
           length=0;
           free(data);//释放内存
           USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//打开接收中断           
         }
        /*********************************/
        /*以下是正常执行动作时的控制代码*/
        /********************************/
         else
         {
         S_Coordinate=change_coordinate(x,y,z);
         setTheta(x,y,z);
     
         runServoTo(servo_ptr,roll);
         
         pitch=pitch+theta2;//结合小臂舵机的角度调整俯仰角，使得尽量平行水平面
         if(pitch>90) pitch=90;
         if(pitch<-90) pitch=-90;
         runServoTo(servo_ptr+1,pitch);
         uint32_t RunTime=getTime();

           if(pinch<60)
           {
            runServoTo(servo_ptr+3,-10);//合爪子
           }
           else runServoTo(servo_ptr+3,60);//张爪子

         
         
         //runServoTo(servo_ptr+2,yaw);
         if(S_Coordinate.r>100)
         {
         runServoTo(servo_ptr+2, -S_Coordinate.phi);//底座舵机         
         }
         else
         {
           runServoTo(servo_ptr+2,yaw);//底座舵机      
           
         }
         if(theta2>90) theta2=90;
         else if(theta2<-90) theta2=-90;
         if(theta1>90) theta1=90;
         else if(theta1<-90) theta1=-90;

         runServoTo(servo_ptr+4,theta1);//大臂舵机
         runServoTo(servo_ptr+5,theta2);//小臂舵机                  
         if( circleGesture>=2)
         {
            DrawCircle(x,y,z);  
           //Nod_Nod();
                     
         }//画圈手势命令
         if(tapGesture==1)
         {
           //Nod_Nod();
         }

         
         armStatus=1;//更新手臂状态
         ready=0;
         length=0;
         free(data);//释放内存
         USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//打开接收中断
         }
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



void armInit(){
    USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//暂时关闭接收中断
  
       for(int x=0;x<6;x++)
       {
         int target=(*servo_ptr).lastPos;
         runServoTo(servo_ptr,target);
         servo_ptr++;
       }
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//打开接收中断

}

struct Spherical_coor change_coordinate(int x,int y,int z){
  struct Spherical_coor temp;
  temp.r=sqrt(x*x+y*y+z*z);
  
  double phi_temp,theta_temp;
  phi_temp=atan((double)x/(double)z)*180/PI;
  theta_temp=acos(y/temp.r)*180/PI;
  temp.phi=(int) phi_temp;
  temp.theta=(int) theta_temp;
  //temp.theta=-temp.theta;   //是phi?
  if(temp.r>190) temp.r=190;//限制长度
  return temp;
}
void setTheta(int x,int y,int z){
 double r=S_Coordinate.r;
 
 theta3=acos((r*r+975)/(200*r))*180/PI;
 theta4=acos((19025-r*r)/19000)*180/PI;
 theta1=(int) S_Coordinate.theta-theta3;
// if(z>-20) theta1=-theta1;
 theta1=-theta1;
 theta2=180-(int) theta4;
}
/************************************************************/
//函数名:runServoToByStep
//输入：舵机结构体指针，目标角度，步长角度，每步时间间隔（ms）
//返回值：无
//描述：根据命令控制某一舵机按照特定步长和时间间隔到达某一角度
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
           break;//手腕的舵机
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

           break;//手腕的俯仰舵机
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
           break;  //底座旋转舵机
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

           break;   //爪子
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
         }  //位置舵机1

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
                  //位置舵机2
           break;           

  }
  
  (*s_ptr).lastPos=target;//更新位置
  

  
}
void runServoTo(struct servo* s_ptr,int target)
{
  int pwm;

  switch ((*s_ptr).label){
  case 1:
          pwm=target*(-19)+3010;
          TIM_SetCompare1 (TIM3,pwm);
        
           break;//手腕的舵机
  case 2: pwm=target*(19)+3010;
          TIM_SetCompare2 (TIM3,pwm);
           break;//手腕的俯仰舵机
  case 3: pwm=target*(-19)+3010;
          TIM_SetCompare3 (TIM3,pwm);
           break;  //底座旋转舵机
  case 4:
           pwm=target*(-19)+3010;
          TIM_SetCompare4 (TIM3,pwm);//爪子
           break;   
  case 5://target+=20;//调整偏移量
           pwm=target*(-19)+3010;
          TIM_SetCompare1 (TIM2,pwm);//位置舵机1
           break;  
  case 6:
           pwm=target*(-19)+3010;
          TIM_SetCompare2 (TIM2,pwm);//位置舵机2
           break;           
  default:pwm=target*19+3010;
           break;
  }
  
  (*s_ptr).lastPos=target;//更新位置
  

  
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
void Nod_Nod(void)//机械臂点头
{
  int memory[6];
  for(int x=0;x<6;x++)
  {
    memory[x]=servos[x].lastPos;
  }//记录动作前的位置
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
     
     }*///恢复到动作前

     
}
void DrawCircle(int cx,int cy,int cz)//机械臂在当前水平面上画圆  
{
  double angle;//0.017453约等于1度
  int X,Y,Z;
  for(angle=0;angle<=2*PI;angle+=0.087)//5度为步长
  {
    X=(int) cx+cos(angle)*radius;
    Z=(int) cz+sin(angle)*radius;
    Y=cy;
    S_Coordinate=change_coordinate(X,Y,Z);
    setTheta(X,Y,Z);
    
    runServoTo(servo_ptr,0);
         
    pitch=pitch+theta2;//结合小臂舵机的角度调整俯仰角，使得尽量平行水平面
    if(pitch>90) pitch=90;
    if(pitch<-90) pitch=-90;
    runServoTo(servo_ptr+1,pitch);


    runServoTo(servo_ptr+3,-10);//合爪子


    runServoTo(servo_ptr+2, -S_Coordinate.phi);//底座舵机         


    if(theta2>90) theta2=90;
    else if(theta2<-90) theta2=-90;
    if(theta1>90) theta1=90;
    else if(theta1<-90) theta1=-90;

    runServoTo(servo_ptr+4,theta1);//大臂舵机
    runServoTo(servo_ptr+5,theta2);//小臂舵机 
    
    Delay_ms(20);
    
  }
}
