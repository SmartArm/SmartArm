//#include"arm.h"
#include"pwm.h"
#include"wifi.h"
#include<stdio.h>
#include<stdlib.h>
#include<stm32f4xx_it.h>
#include<math.h>
#include"SysTick.h"
#define PI 3.141592653
#define dataNum 9   //15
#define l1 100 //大臂长度mm
#define l2 95 //小臂长度mm
char USART1_RECV_BUF[300];
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
  {12,1},{0,2},{0,3},{60,4},{0,5},{0,6}
};
struct Spherical_coor{
  float r;
  int phi;
  int theta;
}S_Coordinate={195,180,0};

struct servo* servo_ptr=&servos[0];  
int theta1=0,theta2=0,theta3=0;//theta1是大臂舵机角度,theta2是小臂舵机角度
int armStatus=1;//手臂状态记录，0代表处于复位状态，1代表正常动作状态
//double theta3=0,theta4=0;
void runServoToByStep(struct servo* s_ptr,int target,int step,int interval);
void runServoTo(struct servo* s_ptr,int target);
void armInit(void);
void armReset(void);
#define radius 50;//用于画圆函数的半径
void DrawCircle(int cx,int cy,int cz);//机械臂在当前水平面上画圆  
void up_and_down(void);//机械臂点头
int valid=0,x=0,y=0,z=0,pitch=90,roll=90,yaw=0,pinch=100, circleGesture=0,tapGesture=0,speed=0;//valid代表当前用户的手是否在识别区里
 uint32_t Time1=0,Time2=0,t=0;
 ////////////////////////////////////////////////
#define L1 10//150
#define L2 100//614
#define L3 10//155
#define L4 -100//-640
#define L6 0//0

//double ino[7];输入的角度值
double nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz;//转换矩阵的值
double outo[9][7];//输出角度值
int pointnum;//记录是第几行的数据
int DrawFlag[2];//判断是否要显示外维空间和内维空间的数据
int ranum[9];//记录正确数据组的个数
int flagg[9];//传递标志位值

double m_ax = 0.0;
double m_ay = 0.0;
double m_az = 0.0;
double m_nx = 0.0;
double m_ny = 0.0;
double m_nz = 0.0;
double m_ino1 = 0.0;
double m_ino2 = 0.0;
double m_ino3 = 0.0;
double m_ino4 = 0.0;
double m_ino5 = 0.0;
double m_ino6 = 0.0;
double m_ox = 0.0;
double m_oy = 0.0;
double m_oz = 0.0;
double m_px = 0.0;
double m_py = 0.0;
double m_pz = 0.0;
double m_OutO11 = 10.01;
double m_OutO12 = 10.01;
double m_OutO13 = 10.01;
double m_OutO14 = 10.01;
double m_OutO15 = 10.01;
double m_OutO16 = 10.01;
double rmax = 40000;//半径最大值
double rmin = 3025;//半径最小值
double r;//确定位置的长度
void RunNijie(void);
void LinePos(void);
int calcAngle(void);//计算舵机角度
int DetAngScal(double aa, double bb, double cc, double dd, double ee);
///////////////////////////////////////////////
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
   
       //uint32_t Time1=0,Time2=0;

       

        while(1){
           //  Time1=getTime();
         servo_ptr=&servos[0];
         int *data;
         if(ready==1){
        
         USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);//暂时关闭接收中断
         
         data=GetIntData();

         valid=*(data);
         x=*(data+1);
         y=*(data+2);
         z=*(data+3);

         pitch=*(data+4);
         roll=*(data+5);
         roll=roll+12;//修正偏差
         yaw=*(data+6);
         pinch=*(data+7);
         speed=*(data+8);//用户手的移动速度
         //circleGesture=*(data+8);
        // tapGesture=*(data+9);
         //speed=*(data+10);//用户手的移动速度
          
       
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
        /***************************在电脑计算的代码***************************/
          theta1=x;
          theta2=y;
          theta3=z;
          if(theta3>90) theta3=90;
          else if(theta3<-80) theta3=-80;
          if(theta2>90) theta2=90;
          else if(theta2<-90) theta2=-90;
          if(theta1>90) theta1=90;
          else if(theta1<-90) theta1=-90;
          

          runServoTo(servo_ptr+2,theta1);//底座舵机 
          runServoTo(servo_ptr+4,theta2);//大臂舵机
          runServoTo(servo_ptr+5,theta3);//小臂舵机      
	
         
         runServoTo(servo_ptr,roll);
         
         pitch=pitch+90-theta3+theta2;//结合小臂舵机的角度调整俯仰角，使得尽量平行水平面
        // pitch=pitch+theta2;
         if(pitch>90) pitch=90;
         if(pitch<-90) pitch=-90;
         runServoTo(servo_ptr+1,pitch);
        

         if(pinch<60)
           {
            runServoTo(servo_ptr+3,-10);//合爪子
           }
         else runServoTo(servo_ptr+3,60);//张爪子
         /****************************在STM32计算的代码***********************************/
        /* px=x;
         py=y;
         pz=z;
         
         if (calcAngle() == 1)//calcAngle返回1说明有解
	{
	  theta1 = (int) -m_OutO11;		
	  theta2 = (int) -m_OutO12 - 90;
	  theta3 = (int) -m_OutO13 + 90;
          if(theta3>90) theta3=90;
          else if(theta3<-80) theta3=-80;
          if(theta2>90) theta2=90;
          else if(theta2<-90) theta2=-90;
          if(theta1>90) theta1=90;
          else if(theta1<-90) theta1=-90;
          
          if((x*x+y*y)>3600)//底面半径平方小于某值时则跳过
         {
          runServoTo(servo_ptr+2,theta1);//底座舵机 
         }
          runServoTo(servo_ptr+4,theta2);//大臂舵机
          runServoTo(servo_ptr+5,theta3);//小臂舵机      
	}
         
         runServoTo(servo_ptr,roll);
         
         //pitch=pitch+90-theta3+theta2;//结合小臂舵机的角度调整俯仰角，使得尽量平行水平面
         pitch=pitch+theta2;
         if(pitch>90) pitch=90;
         if(pitch<-90) pitch=-90;
         runServoTo(servo_ptr+1,pitch);
        

         if(pinch<60)
           {
            runServoTo(servo_ptr+3,-10);//合爪子
           }
         else runServoTo(servo_ptr+3,60);//张爪子
         */

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
/*****************根据收到的直角坐标系进行逆解求解得出各个舵机的角度***********/
void RunNijie()//求逆解
{
	double s1[3], s2[5], s3[5], s4[9], s5[9], c1[3], c2[5], c3[5], c4[9], c5[9], s23[5], c23[5];//这里的s23为sin(o2-o3),c23为cos(o2-o3)
	double u[3], v, a[3], h[5], w[3];//中间变量，省的写很长的式子

	//算o1,有两个解
	outo[1][1] = atan2(ay*L6 + py, ax*L6 + px);
	c1[1] = cos(outo[1][1]);
	s1[1] = sin(outo[1][1]);

	outo[3][1] = outo[1][1];
	outo[5][1] = outo[1][1];
	outo[7][1] = outo[1][1];

	outo[2][1] = atan2(-ay*L6 - py, -ax*L6 - px);
	c1[2] = cos(outo[2][1]);
	s1[2] = sin(outo[2][1]);

	outo[4][1] = outo[2][1];
	outo[6][1] = outo[2][1];
	outo[8][1] = outo[2][1];

	//算o2，由于o1有两个解，o2有四个解,
	u[1] = c1[1] * (ax*L6 + px) + s1[1] * (ay*L6 + py) - L1;
	v = az*L6 + pz;
	a[1] = (L3*L3 + L4*L4 - L2*L2 - u[1] * u[1] - v*v) / (2 * L2);

	u[2] = c1[2] * (ax*L6 + px) + s1[2] * (ay*L6 + py) - L1;
	a[2] = (L3*L3 + L4*L4 - L2*L2 - u[2] * u[2] - v*v) / (2 * L2);

	w[1] = u[1] * u[1] + v*v - a[1] * a[1];
	w[2] = u[2] * u[2] + v*v - a[2] * a[2];

	outo[1][2] = atan2(a[1] * v - u[1] * sqrt(w[1]), -1 * a[1] * u[1] - v*sqrt(w[1]));
	c2[1] = cos(outo[1][2]);
	s2[1] = sin(outo[1][2]);

	//  if ( (-1*a[2]*u[2]-v*sqrt(u[2]*u[2]+v*v-a[2]*a[2]) )!=0 )
	//  {
	//  }

	outo[2][2] = atan2(a[2] * v - u[2] * sqrt(w[2]), -1 * a[2] * u[2] - v*sqrt(w[2]));
	c2[2] = cos(outo[2][2]);
	s2[2] = sin(outo[2][2]);


	outo[3][2] = atan2(a[1] * v + u[1] * sqrt(w[1]), -1 * a[1] * u[1] + v*sqrt(w[1]));
	c2[3] = cos(outo[3][2]);
	s2[3] = sin(outo[3][2]);

	//  if ( (-1*a[2]*u[2]+v*sqrt(u[2]*u[2]+v*v-a[2]*a[2]) )!=0 )
	//  {
	//  }
	outo[4][2] = atan2(a[2] * v + u[2] * sqrt(w[2]), -1 * a[2] * u[2] + v*sqrt(w[2]));
	c2[4] = cos(outo[4][2]);
	s2[4] = sin(outo[4][2]);

	outo[5][2] = outo[1][2];
	outo[6][2] = outo[2][2];
	outo[7][2] = outo[3][2];
	outo[8][2] = outo[4][2];


	//算o3,完全取决于o2，所以有四种情况。
	outo[1][3] = atan2((s2[1] * u[1] + c2[1] * v)*L3 + (L2 + s2[1] * v - c2[1] * u[1])*L4, (s2[1] * u[1] + c2[1] * v)*L4 - (L2 + s2[1] * v - c2[1] * u[1])*L3);
	s3[1] = sin(outo[1][3]);
	c3[1] = cos(outo[1][3]);

	outo[2][3] = atan2((s2[2] * u[2] + c2[2] * v)*L3 + (L2 + s2[2] * v - c2[2] * u[2])*L4, (s2[2] * u[2] + c2[2] * v)*L4 - (L2 + s2[2] * v - c2[2] * u[2])*L3);
	s3[2] = sin(outo[2][3]);
	c3[2] = cos(outo[2][3]);

	outo[3][3] = atan2((s2[3] * u[1] + c2[3] * v)*L3 + (L2 + s2[3] * v - c2[3] * u[1])*L4, (s2[3] * u[1] + c2[3] * v)*L4 - (L2 + s2[3] * v - c2[3] * u[1])*L3);
	s3[3] = sin(outo[3][3]);
	c3[3] = cos(outo[3][3]);

	//  if ( ((s2[4]*u[2]+c2[4]*v)*L4 - (L2+s2[4]*v-c2[4]*u[2])*L3)!=0 )
	//  {
	//  }
	outo[4][3] = atan2((s2[4] * u[2] + c2[4] * v)*L3 + (L2 + s2[4] * v - c2[4] * u[2])*L4, (s2[4] * u[2] + c2[4] * v)*L4 - (L2 + s2[4] * v - c2[4] * u[2])*L3);
	s3[4] = sin(outo[4][3]);
	c3[4] = cos(outo[4][3]);

	outo[5][3] = outo[1][3];
	outo[6][3] = outo[2][3];
	outo[7][3] = outo[3][3];
	outo[8][3] = outo[4][3];



	//算o5,八个解，取决于o1o2o3.
	s23[1] = s2[1] * c3[1] - c2[1] * s3[1];
	s23[2] = s2[2] * c3[2] - c2[2] * s3[2];
	s23[3] = s2[3] * c3[3] - c2[3] * s3[3];
	s23[4] = s2[4] * c3[4] - c2[4] * s3[4];

	c23[1] = c2[1] * c3[1] + s2[1] * s3[1];
	c23[2] = c2[2] * c3[2] + s2[2] * s3[2];
	c23[3] = c2[3] * c3[3] + s2[3] * s3[3];
	c23[4] = c2[4] * c3[4] + s2[4] * s3[4];


	h[1] = -1 * s23[1] * (ax*c1[1] + ay*s1[1]) - az*c23[1];
	h[2] = -1 * s23[2] * (ax*c1[2] + ay*s1[2]) - az*c23[2];
	h[3] = -1 * s23[3] * (ax*c1[1] + ay*s1[1]) - az*c23[3];
	h[4] = -1 * s23[4] * (ax*c1[2] + ay*s1[2]) - az*c23[4];


	outo[1][5] = atan2(sqrt(1 - h[1] * h[1]), h[1]);
	s5[1] = sin(outo[1][5]);
	c5[1] = cos(outo[1][5]);

	outo[2][5] = atan2(sqrt(1 - h[2] * h[2]), h[2]);
	s5[2] = sin(outo[2][5]);
	c5[2] = cos(outo[2][5]);

	outo[3][5] = atan2(sqrt(1 - h[3] * h[3]), h[3]);
	s5[3] = sin(outo[3][5]);
	c5[3] = cos(outo[3][5]);

	outo[4][5] = atan2(sqrt(1 - h[4] * h[4]), h[4]);
	s5[4] = sin(outo[4][5]);
	c5[4] = cos(outo[4][5]);

	outo[5][5] = atan2(-sqrt(1 - h[1] * h[1]), h[1]);
	s5[5] = sin(outo[5][5]);
	c5[5] = cos(outo[5][5]);

	outo[6][5] = atan2(-sqrt(1 - h[2] * h[2]), h[2]);
	s5[6] = sin(outo[6][5]);
	c5[6] = cos(outo[6][5]);

	outo[7][5] = atan2(-sqrt(1 - h[3] * h[3]), h[3]);
	s5[7] = sin(outo[7][5]);
	c5[7] = cos(outo[7][5]);

	outo[8][5] = atan2(-sqrt(1 - h[4] * h[4]), h[4]);
	s5[8] = sin(outo[8][5]);
	c5[8] = cos(outo[8][5]);

	//算o4
	//  if (((c23[1]*(ax*c1[1]+ay*s1[1])-az*s23[1])/s5[1])!=0&&s5[1]!=0)
	//  {
	//  }
	outo[1][4] = atan2(-(ax*s1[1] - ay*c1[1]) / s5[1], (c23[1] * (ax*c1[1] + ay*s1[1]) - az*s23[1]) / s5[1]);
	s4[1] = sin(outo[1][4]);
	c4[1] = cos(outo[1][4]);

	outo[2][4] = atan2(-(ax*s1[2] - ay*c1[2]) / s5[2], (c23[2] * (ax*c1[2] + ay*s1[2]) - az*s23[2]) / s5[2]);
	s4[2] = sin(outo[2][4]);
	c4[2] = cos(outo[2][4]);

	outo[3][4] = atan2(-(ax*s1[1] - ay*c1[1]) / s5[3], (c23[3] * (ax*c1[1] + ay*s1[1]) - az*s23[3]) / s5[3]);
	s4[3] = sin(outo[3][4]);
	c4[3] = cos(outo[3][4]);

	outo[4][4] = atan2(-(ax*s1[2] - ay*c1[2]) / s5[4], (c23[4] * (ax*c1[2] + ay*s1[2]) - az*s23[4]) / s5[4]);
	s4[4] = sin(outo[4][4]);
	c4[4] = cos(outo[4][4]);

	//  if (((c23[1]*(ax*c1[1]+ay*s1[1])-az*s23[1])/s5[5])!=0&&s5[1]!=0)
	//  {
	//  }
	outo[5][4] = atan2(-(ax*s1[1] - ay*c1[1]) / s5[5], (c23[1] * (ax*c1[1] + ay*s1[1]) - az*s23[1]) / s5[5]);
	s4[5] = sin(outo[5][4]);
	c4[5] = cos(outo[5][4]);

	outo[6][4] = atan2(-(ax*s1[2] - ay*c1[2]) / s5[6], (c23[2] * (ax*c1[2] + ay*s1[2]) - az*s23[2]) / s5[6]);
	s4[6] = sin(outo[6][4]);
	c4[6] = cos(outo[6][4]);

	outo[7][4] = atan2(-(ax*s1[1] - ay*c1[1]) / s5[7], (c23[3] * (ax*c1[1] + ay*s1[1]) - az*s23[3]) / s5[7]);
	s4[7] = sin(outo[7][4]);
	c4[7] = cos(outo[7][4]);

	outo[8][4] = atan2(-(ax*s1[2] - ay*c1[2]) / s5[8], (c23[4] * (ax*c1[2] + ay*s1[2]) - az*s23[4]) / s5[8]);
	s4[8] = sin(outo[8][4]);
	c4[8] = cos(outo[8][4]);

	//算o6
	//  outo[1][6] = atan2(s4[1]*c5[1]*(c1[1]*oy-s1[1]*ox)-c4[1]*(s1[1]*nx-c1[1]*ny),(s1[1]*nx-c1[1]*ny)*(s1[1]*nx-c1[1]*ny)+(s1[1]*ox-c1[1]*oy)*(s1[1]*ox-c1[1]*oy));
	outo[1][6] = atan2(s4[1] * c5[1] * (c1[1] * oy - s1[1] * ox) - c4[1] * (s1[1] * nx - c1[1] * ny), s4[1] * c5[1] * (c1[1] * ny - s1[1] * nx) + c4[1] * (s1[1] * ox - c1[1] * oy));


	outo[2][6] = atan2(s4[2] * c5[2] * (c1[2] * oy - s1[2] * ox) - c4[2] * (s1[2] * nx - c1[2] * ny), s4[2] * c5[2] * (c1[2] * ny - s1[2] * nx) + c4[2] * (s1[2] * ox - c1[2] * oy));

	outo[3][6] = atan2(s4[3] * c5[3] * (c1[1] * oy - s1[1] * ox) - c4[3] * (s1[1] * nx - c1[1] * ny), s4[3] * c5[3] * (c1[1] * ny - s1[1] * nx) + c4[3] * (s1[1] * ox - c1[1] * oy));

	outo[4][6] = atan2(s4[4] * c5[4] * (c1[2] * oy - s1[2] * ox) - c4[4] * (s1[2] * nx - c1[2] * ny), s4[4] * c5[4] * (c1[2] * ny - s1[2] * nx) + c4[4] * (s1[2] * ox - c1[2] * oy));

	outo[5][6] = atan2(s4[5] * c5[5] * (c1[1] * oy - s1[1] * ox) - c4[5] * (s1[1] * nx - c1[1] * ny), s4[5] * c5[5] * (c1[1] * ny - s1[1] * nx) + c4[5] * (s1[1] * ox - c1[1] * oy));

	outo[6][6] = atan2(s4[6] * c5[6] * (c1[2] * oy - s1[2] * ox) - c4[6] * (s1[2] * nx - c1[2] * ny), s4[6] * c5[6] * (c1[2] * ny - s1[2] * nx) + c4[6] * (s1[2] * ox - c1[2] * oy));

	outo[7][6] = atan2(s4[7] * c5[7] * (c1[1] * oy - s1[1] * ox) - c4[7] * (s1[1] * nx - c1[1] * ny), s4[7] * c5[7] * (c1[1] * ny - s1[1] * nx) + c4[7] * (s1[1] * ox - c1[1] * oy));

	outo[8][6] = atan2(s4[8] * c5[8] * (c1[2] * oy - s1[2] * ox) - c4[8] * (s1[2] * nx - c1[2] * ny), s4[8] * c5[8] * (c1[2] * ny - s1[2] * nx) + c4[8] * (s1[2] * ox - c1[2] * oy));
}

void LinePos()
{
	// TODO: Add your control notification handler code here
	double zo, yo, czo, szo, cyo, syo;

	zo = atan2(py, px);
	szo = sin(zo);
	czo = cos(zo);

	yo = atan2(sqrt(px*px + py*py), pz);
	syo = sin(yo);
	cyo = cos(yo);


	nx = cyo*czo;
	ny = szo*cyo;
	nz = -syo;
	ox = -szo;
	oy = czo;
	oz = 0;
	ax = syo*czo;
	ay = szo*syo;
	az = cyo;

	m_nx = nx;
	m_ny = ny;
	m_nz = nz;

	m_ox = ox;
	m_oy = oy;
	m_oz = oz;

	m_ax = ax;
	m_ay = ay;
	m_az = az;

}
//限制范围的函数
int DetAngScal(double aa, double bb, double cc, double dd, double ee)
{
	int i = 0;

	if (aa<-90 || aa>90)
	{
		i = 1;
	}

	if (bb<-180 || bb>0)
	{
		i = 1;
	}

	if (cc<-10 || cc>180)
	{
		i = 1;
	}
	/*
	if (dd<-180 || dd>180)
	{
		i = 1;
	}

	if (ee<-135 || ee>0)
	{
	i = 1;
	}
	*/
	if (i)
	{
		return 0;
	}

	return 1;
}

int calcAngle(void)//计算舵机角度()
{
	// TODO: Add your control notification handler code here
	nx = m_nx;
	ny = m_ny;
	nz = m_nz;

	ox = m_ox;
	oy = m_oy;
	oz = m_oz;

	ax = m_ax;
	ay = m_ay;
	az = m_az;

	
	r = px*px + py*py + pz*pz;
	
	
	if (r<rmin || r>rmax)
	{
		return 0;
	}
	else
	{
		LinePos();//线性化
		RunNijie();

		m_OutO11 = outo[1][1] * 180 / PI;
		m_OutO12 = outo[1][2] * 180 / PI;
		m_OutO13 = outo[1][3] * 180 / PI;
		m_OutO14 = outo[1][4] * 180 / PI;
		m_OutO15 = outo[1][5] * 180 / PI;
		m_OutO16 = outo[1][6] * 180 / PI;
		
		if (!DetAngScal(m_OutO11, m_OutO12, m_OutO13, m_OutO14, m_OutO15))
		{
			return 0;//此位置无解
		}
		else
		{
			return 1;//此位置有解
		}
		
		//return 1;
		
	}
	/*
	m_OutO21 = outo[2][1] * 180 / PI;
	m_OutO22 = outo[2][2] * 180 / PI;
	m_OutO23 = outo[2][3] * 180 / PI;
	m_OutO24 = outo[2][4] * 180 / PI;
	m_OutO25 = outo[2][5] * 180 / PI;
	m_OutO26 = outo[2][6] * 180 / PI;
	for (int i = 1; i <= 7; i++)
	{
	cout << outo[2][i] << " ";
	}
	cout << endl;
	cout << m_OutO21 << " " << m_OutO22 << " " << m_OutO23 << " " << m_OutO24 << " " << m_OutO25 << " " << m_OutO26 << " " << endl;

	m_OutO31 = outo[1][1] * 180 / PI;
	m_OutO32 = outo[3][2] * 180 / PI;
	m_OutO33 = outo[3][3] * 180 / PI;
	m_OutO34 = outo[3][4] * 180 / PI;
	m_OutO35 = outo[3][5] * 180 / PI;
	m_OutO36 = outo[3][6] * 180 / PI;

	m_OutO41 = outo[2][1] * 180 / PI;
	m_OutO42 = outo[4][2] * 180 / PI;
	m_OutO43 = outo[4][3] * 180 / PI;
	m_OutO44 = outo[4][4] * 180 / PI;
	m_OutO45 = outo[4][5] * 180 / PI;
	m_OutO46 = outo[4][6] * 180 / PI;

	m_OutO51 = outo[1][1] * 180 / PI;
	m_OutO52 = outo[1][2] * 180 / PI;
	m_OutO53 = outo[1][3] * 180 / PI;
	m_OutO54 = outo[5][4] * 180 / PI;
	m_OutO55 = outo[5][5] * 180 / PI;
	m_OutO56 = outo[5][6] * 180 / PI;

	m_OutO61 = outo[2][1] * 180 / PI;
	m_OutO62 = outo[2][2] * 180 / PI;
	m_OutO63 = outo[2][3] * 180 / PI;
	m_OutO64 = outo[6][4] * 180 / PI;
	m_OutO65 = outo[6][5] * 180 / PI;
	m_OutO66 = outo[6][6] * 180 / PI;

	m_OutO71 = outo[1][1] * 180 / PI;
	m_OutO72 = outo[3][2] * 180 / PI;
	m_OutO73 = outo[3][3] * 180 / PI;
	m_OutO74 = outo[7][4] * 180 / PI;
	m_OutO75 = outo[7][5] * 180 / PI;
	m_OutO76 = outo[7][6] * 180 / PI;


	m_OutO81 = outo[2][1] * 180 / PI;
	m_OutO82 = outo[4][2] * 180 / PI;
	m_OutO83 = outo[4][3] * 180 / PI;
	m_OutO84 = outo[8][4] * 180 / PI;
	m_OutO85 = outo[8][5] * 180 / PI;
	m_OutO86 = outo[8][6] * 180 / PI;
	*/
}

/******************************************************************************/
