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
  {12,1},{0,2},{0,3},{60,4},{40,5},{0,6}
};
struct Spherical_coor{
  float r;
  int phi;
  int theta;
}S_Coordinate={195,180,0};

struct servo* servo_ptr=&servos[0];  
int theta1=0,theta2=0;//theta1是大臂舵机角度,theta2是小臂舵机角度
int armStatus=1;//手臂状态记录，0代表处于复位状态，1代表正常动作状态

void runServoToByStep(struct servo* s_ptr,int target,int step,int interval);
void runServoTo(struct servo* s_ptr,int target);
void armInit(void);
//void armReset(void);
#define radius 50;//用于画圆函数的半径
void DrawCircle(int cx,int cy,int cz);//机械臂在当前水平面上画圆  
void up_and_down(void);//机械臂点头
int Mode=1,valid=0,x=0,y=0,z=0,pitch=90,roll=90,yaw=0,pinch=100, circleGesture=0,tapGesture=0,speed=0;//Mode代表进入什么控制模式，1代表手动控制，2代表自动追踪
//valid代表当前用户的手是否在识别区里
//uint32_t Time1=0,Time2=0,t=0;
void angleCalc(int _x,int _y,int _z,int _pitch);//计算舵机的角度
void controlServo(int t1,int t2,int t3,int t4,int t5,int t6);//执行控制命令
/*********************************以下是自动追踪PID算法的函数************************************************/
#define CENTERX 320
#define CENTERY 240
int First=1;
typedef struct PID
{
  int SetPoint; //设定目标 Desired Value
  long SumError; //误差累计
  double Proportion;//比例常数 Proportional Const
  double Integral; //积分常数 Integral Const
  double Derivative; //微分常数 Derivative Const
  int LastError; //Error[-1]
  int PrevError; //Error[-2]
};
u8 PIDTime=0;//当该变量为1时，说明计数时间到，需要进行PID计算
static struct PID xPID={0,0,0,0,0,0,0},yPID={0,0,0,0,0,0,0};//初始化两个PID结构体
static struct PID *Xptr = &xPID;
static struct PID *Yptr = &yPID;
int outputX=0,outputY=200,outputZ=100;//初始化机械臂到合理的姿态
//增量式PID控制设计
int IncPIDCalc(int NextPoint,struct PID * ptr)
{
  register int iError, iIncpid;
  //当前误差
  iError = ptr->SetPoint - NextPoint;
  //增量计算
  iIncpid = ptr->Proportion * iError //E[k]项
          - ptr->Integral * ptr->LastError //E[k－1]项
          + ptr->Derivative * ptr->PrevError; //E[k－2]项
  //存储误差，用于下次计算
  ptr->PrevError = ptr->LastError;
  ptr->LastError = iError;
  //返回增量值
  return iIncpid;
}
/************************设置比例参数*************************/
void SetProportion(struct PID * ptr,double p)
{
  ptr->Proportion=p;
}
/************************设置微分参数*************************/
void SetDerivative(struct PID * ptr,double d)
{
  ptr->Derivative=d;
}
/************************设置目标*************************/
void SetTarget(struct PID * ptr,double target)
{
  ptr->SetPoint=target;
}
/*********************************PID算法的函数代码段结束************************************************/
int main(void)
{
       NVIC_Config();
       USART1_Config();
       TIM3_PWM_Init();
       TIM2_PWM_Init();
       TIM1_GPIO_Config();
       Tim1_Config();
       TIM4_Config();//PID定时
       //TIM_SetCompare1 (TIM1,10000);
       //初始化串口1，中断方式接收
       SysTick_Init();//初始化时钟
       SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;//使能时钟

       armInit();//初始化机械臂
      // runServoTo(servo_ptr+4,120);//大臂舵机,当输入40度时，刚好垂直，需要校正这个偏移量
      //  runServoTo(servo_ptr+4,-60);//大臂舵机,当输入40度时，刚好垂直，需要校正这个偏移量
       //runServoTo(servo_ptr+3,60);//大臂舵机
      // runServoTo(servo_ptr+3,-10);//大臂舵机
       
       //runServoTo(servo_ptr+3,-90);//大臂舵机
       SetTarget(Xptr,CENTERX);
       SetTarget(Yptr,CENTERY);//将追踪的PID算法的目标设置为中心点




        while(1){
         //Time1=getTime();
         servo_ptr=&servos[0];
         int *data;
         if(ready==1){//数据准备好
        
         USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//暂时关闭接收中断
         
         data=GetIntData();

         Mode=*(data);//先读取模式
         if(Mode=1)//如果进入手动控制模式
         {
           int First=1;//将PID算法中的首次标志置为1
           valid=*(data+1);
           x=*(data+2);
           y=*(data+3);
           z=*(data+4);
           //if((z>-90&&z<200)&&(x>-200&&x<200)&&(y>80&&y<400)) {
             
           y=y-50;//将坐标系沿Y轴平移，方便控制
           z=z-200;//将坐标系沿Z轴平移，方便控制
           if(z>0) z=0;//限制边界
           pitch=*(data+5);
           roll=*(data+6);
           roll=roll+12;//修正偏差
           yaw=*(data+7);
           pinch=*(data+8);
           circleGesture=*(data+9);
           tapGesture=*(data+10);
           speed=*(data+11);//用户手的移动速度
            
  
           if(valid==0)
           {
             ready=0;
             length=0;
             free(data);//释放内存
             USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//打开接收中断  
             continue;
           }
             /*********************************/
             /*以下是正常执行动作时的控制代码*/
             /********************************/
              else
              {
              
              if(speed<10)
              {
                  armStatus=1;//更新手臂状态
                  ready=0;
                  length=0;
                  free(data);//释放内存
                  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//打开接收中断
                  continue;
              
              }//当手的速度过小，认为是静止的
         
              angleCalc(x,y,z,pitch);//计算角度
              int clawAngle;
              if(pinch<60)
              {
                  clawAngle=-10;
                 //合爪子
                }
              else  clawAngle=60;
     
     
              controlServo(roll,pitch,yaw,clawAngle,theta1,theta2);//执行控制命令 //t1~t6分别对应手腕ROLL,手腕PITCH，底座角度，爪子，大臂，小臂
               
             /* if( circleGesture>=2)
              {
                 DrawCircle(x,y,z);  
                
     
                          
              }//画圈手势命令
              if(tapGesture==1)
              {
                
                 up_and_down();
              }
            */
        
              armStatus=1;//更新手臂状态
              ready=0;
              length=0;
              free(data);//释放内存
               
              USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//打开接收中断
       
         
         }
         
        }
        else if(Mode==2)
         {
           if(First==1)
           {
            //第一次进入,将舵机调整到合理的位置
            angleCalc(outputX,outputY,outputZ,pitch);//计算角度
            controlServo(0,pitch,-S_Coordinate.phi,60,theta1,theta2);//执行控制命令 //t1~t6分别对应手腕ROLL,手腕PITCH，底座角度，爪子，大臂，小臂
            Delay_ms(500);
            First=0;
           }
           //进入自动追踪模式,每0.1S执行一次PID计算
           if(PIDTime==1)
           {
             int Pause,CamX,CamY;//追踪是否暂停，被追踪物体中心在屏幕的位置
             Pause=*(data+1);
             CamX=*(data+2);
             CamY=*(data+3);//获取数据

             outputX+=IncPIDCalc(CamX,Xptr);
             outputY+=IncPIDCalc(CamY,Yptr);
             
             PIDTime=0;
             TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE); //打开TIM4 中断 
           }
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
//****************************以下是计算位置的函数*****************************/
//计算角度的辅助函数
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
 double angleTemp1=0,angleTemp2=0;
 angleTemp1=acos((r*r+975)/(200*r))*180/PI;
 angleTemp2=acos((19025-r*r)/19000)*180/PI;
 theta1=(int) S_Coordinate.theta-angleTemp1;
// if(z>-20) theta1=-theta1;
 theta1=-theta1;
 theta2=180-(int) angleTemp2;
}
//***********************************根据X,Y,Z计算出大臂舵机和小臂舵机的角度*************************/
void angleCalc(int _x,int _y,int _z,int _pitch)
{
   S_Coordinate=change_coordinate(_x,_y,_z);
   setTheta(_x,_y,_z);
   pitch=_pitch+theta2;//结合小臂舵机的角度调整俯仰角，使得尽量平行水平面
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
//t1~t6分别对应手腕ROLL,手腕PITCH，底座角度，爪子，大臂，小臂
void controlServo(int t1,int t2,int t3,int t4,int t5,int t6)
{
   t5=t5+40;//修正大臂舵机的偏移量
   if(t2>90) t2=90;
   else if(t2<-90) t2=-90;
   if(t5>120) t5=120;
   else if(t5<-60) t5=-60;
   if(t6>90) t6=90;
   else if(t6<-90) t6=-90;//限制角度

   runServoTo(servo_ptr,t1);
   runServoTo(servo_ptr+1,t2);
   if(S_Coordinate.r>100)
    {
           runServoTo(servo_ptr+2, -S_Coordinate.phi);//底座舵机         
    }
    else
    {
           runServoTo(servo_ptr+2,yaw);//底座舵机      
    }
   runServoTo(servo_ptr+2,t3);//底座舵机    
   runServoTo(servo_ptr+3,t4);//合爪子
   runServoTo(servo_ptr+4,t5);//大臂舵机
   runServoTo(servo_ptr+5,t6);//小臂舵机      
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
           break;//手腕的舵机
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
           TIM_SetCompare4(TIM3,pwm);//爪子
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
  for(int i=0;i<5;i++)
  {
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

