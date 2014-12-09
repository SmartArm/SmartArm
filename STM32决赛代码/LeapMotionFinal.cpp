
#include <windows.h>//如果嫌刷屏太快打开这个定义1（一共2处）
#include "Leap.h"
#include<sstream>
#include<fstream>
#include <stdio.h>
#include<iostream>
#include <time.h>
#pragma comment(lib,"WS2_32.lib")
using namespace Leap;
using namespace std;
ofstream outFile;
char buff[150];
bool TAP = false;//主程序中用于判断tap手势是否被触发
void getData(const Controller& controller);
void GetHandDirection(const Controller& controller);
void getfingerposition(const Controller& controller);
float indexFinPosX, indexFinPosY;
Leap::Vector startPoint, endPoint;
bool clenchFist = false;//左手是否握拳的标志
int selecting = 0;//标志框选状态，0代表在手动控制模式下，1代表开始框选，2代表框选中，3代表框选完
clock_t startTime, endTime;
int Mode = 1;//模式选择
int normalPause = 0;//手动模式下的暂停
int TrackingPause = 0;//追踪模式下的暂停
//int Tracking = 0;
int tapOrNot = 0;
int camX = 0, camY = 0;
int palmX, palmY, palmZ;
int pitch_int, roll_int, yaw_int;
int speed;
//int theta1, theta2, theta3;
int holdDis = 0;
int circleCount = 0;
int main() {


	Controller controller;

	WSADATA wsd;
	SOCKET sockClient;                                            //客户端socket
	SOCKADDR_IN addrSrv;


	sockClient = socket(AF_INET, SOCK_STREAM, 0);                    //创建socket
	addrSrv.sin_addr.S_un.S_addr = inet_addr("192.168.1.1");
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(8080);
	connect(sockClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));    //连接服务器端

	while (1)
	{
		getData(controller);
		if (TAP)
		{
			for (int i = 0; i < 3; i++)
			{
				send(sockClient, buff, sizeof(buff), 0);
				Sleep(10);//连发三次
			}
			TAP = false;
			tapOrNot = 0;
		}
		send(sockClient, buff, sizeof(buff), 0);
		//system("pause");
		Sleep(10);
	}

	closesocket(sockClient);                                    //关闭连接
	WSACleanup();

	return 0;
}
int valid = 0;
void getData(const Controller& controller)
{
	controller.enableGesture(Gesture::TYPE_CIRCLE);//打开圆圈手势
	controller.enableGesture(Gesture::TYPE_KEY_TAP);
	controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
	controller.enableGesture(Gesture::TYPE_SWIPE);
	//int  progress=0, tapOrNot = 0, speed = 0,distance=0;

	const Frame frame = controller.frame();
	if (!frame.hands().isEmpty()) {
		Hand rightHand, leftHand;
		for (int x = 0; x < frame.hands().count(); x++)
		{
			if (frame.hands()[x].isRight())
			{
				rightHand = frame.hands()[x];
			}//获取右手
			else
			{
				leftHand = frame.hands()[x];//
				
			}//获取左手
		}
		//*****************************************************左手相关数据监测*************************************************************//
		if (leftHand .isValid())
		{

			const Vector palmPosL = leftHand.palmPosition();//手掌中心位置
			const FingerList leftFingers = leftHand.fingers();//获取左手手指列表
			int distance1 = 0;
			int nOfFingers1 = leftFingers.count();
			for (int i = 0; i < nOfFingers1; i++)
			{
				distance1 += leftFingers[i].tipPosition().distanceTo(palmPosL);
			}
			distance1 = (int)(distance1 / nOfFingers1);//计算各个指尖到手心的平均距离
			//cout << distance1 << endl;
			if ((clenchFist == false) && (distance1 < 70))//握拳
			{
				clenchFist = true;
				startTime = clock();
			}
			else if (clenchFist == true && distance1>80)
			{
				clenchFist = false;
				endTime = clock();
				int duration;
				duration = endTime - startTime;
				cout << "握拳时间长度为：" << duration << endl;
				if (duration >= 1000){
					if (Mode == 1){
						Mode = 2;
						selecting = 1;
						cout << "切换至自动追踪模式" << endl;
						cout << "开始框选" << endl;
					}
					else if (Mode == 2)
					{
						Mode = 1;
						cout << "切换至手动控制模式" << endl;
					}
				   
				}
				else cout << "时间不够长" << endl;
				system("pause");
			}
		}
		//*****************************************************右手相关数据监测*************************************************************//
		if (rightHand.isValid())
		{

			const Vector direction = rightHand.direction();//手掌方向
			const Vector wristPos = rightHand.wristPosition();//手腕位置
			const Vector palmnormal = rightHand.palmNormal();//手掌法向量
			const Vector palmPos = rightHand.palmPosition();//手掌中心位置
			Vector handSpeed = rightHand.palmVelocity();
			
			palmX = int(palmPos.x);
			palmY = int(palmPos.y);
			palmZ = int(palmPos.z);
			
			valid = 0;
			
			//px = -(-409.7 + 409.6*cos(palmPos.z*0.002325) + 449.8*sin(palmPos.z*0.002325));
			//py = -(-409.7 + 409.6*cos(palmPos.x*0.002325) + 449.8*sin(palmPos.x*0.002325));
			//pz = -409.7 + 409.6*cos(palmPos.y*0.002325) + 449.8*sin(palmPos.y*0.002325);

			//cout << "x:" << wristX << "y:" << wristY << "z:" << wristZ << "r:" << sqrt((wristX*wristX + wristY*wristY + wristZ*wristZ)) << endl;
			//Sleep(200);

			speed= (int)sqrt((handSpeed.x*handSpeed.x) + (handSpeed.y*handSpeed.y) + (handSpeed.z*handSpeed.z));

			
			pitch_int = int(direction.pitch() * RAD_TO_DEG);
			roll_int = int(rightHand.palmNormal().roll() * RAD_TO_DEG);
			yaw_int = int(direction.yaw() * RAD_TO_DEG);
			// Calculate the hand's pitch, roll, and yaw angles



			const FingerList fingers = rightHand.fingers();

			if (!fingers.isEmpty()) {
				// 获取右手食指的指尖位置

				Vector indexFingerPos;
				indexFingerPos = fingers[1].tipPosition();
				indexFinPosX = indexFingerPos.x;
				indexFinPosY = indexFingerPos.y;


				
				int nOfFingers = fingers.count();
				for (int i = 0; i < nOfFingers; i++)
				{
					holdDis += fingers[i].tipPosition().distanceTo(palmPos);
				}
				holdDis = (int)(holdDis / nOfFingers);//计算各个指尖到手心的平均距离，判断是否合爪子的依据
				//////////////////////////////////////////////////////////////////////////
				//以下代码是用以框选追踪对象的
				int pinch = 0;
				Vector thumbPos = fingers[0].tipPosition();
				pinch = indexFingerPos.distanceTo(thumbPos);//计算拇指到食指的距离
				//cout << pinch << endl;
				if (pinch < 50 && selecting == 1)
				{
					startPoint = indexFingerPos;
					cout << "开始框选，起点为" << startPoint.x << "," << startPoint.y << endl;
					selecting = 2;
					system("pause");
				}
				else if (pinch>70 && selecting == 2)
				{
					endPoint = indexFingerPos;
					cout << "结束框选，终点为" << endPoint.x << "," << endPoint.y << endl;
					selecting = 3;
					system("pause");
				}
			}


		}
		//*****************************************************************以下是手势部分**********************************************************//
		const GestureList gestures = frame.gestures();

		for (int g = 0; g < gestures.count(); ++g) {
			Gesture gesture = gestures[g];

			if (gesture.type() == Gesture::TYPE_CIRCLE) {

				CircleGesture circle = gesture;

				if (circle.state() != Gesture::STATE_START) {
					//cout << "当前转了" << circle.progress() << "圈" << endl;

				}
				circleCount = int(circle.progress());
			}
			if (gesture.type() == Gesture::TYPE_KEY_TAP) {

				KeyTapGesture tap = gesture;
				tapOrNot = (int)tap.progress();
				cout << "tap状态：" << tapOrNot << endl;
				TAP = true;
				
			}
			if (gesture.type() == Gesture::TYPE_SWIPE){
				SwipeGesture swg = gesture;
				HandList handsForGesture = gesture.hands();
				if (handsForGesture.count() == 1 && handsForGesture[0].isLeft())//只有左手在做这个动作
				{
					if (Mode == 1)//在手动控制模式下左手挥动意味着切换手动暂停状态
					{
						if (normalPause == 0)
						{
							normalPause = 1;
							Sleep(100);
						}
						else if (normalPause == 1)
						{
							normalPause = 0;
							Sleep(100);
						}
					}
					else if (Mode == 2)//在自动追踪模式下挥动左手意味着重新框选追踪物
					{
						selecting = 1;//重新框选
						Sleep(100);
						cout << "重新框选" << endl;
						system("pause");
					}
					
				}
				else if (handsForGesture.count() == 1 && handsForGesture[0].isRight())//只有右手在做这个动作
				{
					/*if (Tracking == 1)
					{
						Tracking = 0;
						Sleep(100);
					}*/
				}
			}
			/*if (gesture.type() == Gesture::TYPE_SCREEN_TAP){
				ScreenTapGesture stg = gesture;
				switch (selecting)
				{
				case 0:break;//不需要框选
				case 1://开始框选
					
					startPoint = stg.position();//存储开始点
					cout << "开始框选，起点为" << startPoint.x << "," << startPoint.y<<endl;
					selecting = 2;
					Sleep(100);
					system("pause");
					break;
				case 2://选择结束点
					endPoint = stg.position();//存储结束点
					cout << "结束框选，终点为" << endPoint.x << "," << endPoint.y<<endl;
					selecting = 0;
					Sleep(100);
					system("pause");
					break;
				}
				
				//cout << "检测到SCREENTAP手势，坐标为"<<stg.position().x << " " << stg.position().y<<endl;
				
			}*/
		}
		//sprintf_s(buff, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d \r\n", valid, theta1, theta2, theta3, pitch_int, roll_int, yaw_int, holdDis, progress, tapOrNot, speed, normalPause, Tracking, camX, camY);
		if (Mode == 1)//手动控制模式发送的数据
		{
			sprintf_s(buff, "%d %d %d %d %d %d %d %d %d %d %d %d \r\n", Mode, valid, palmX, palmY, palmZ, pitch_int, roll_int, yaw_int, holdDis, tapOrNot, circleCount, speed);
			cout << buff << endl;
		}
		else if (Mode == 2)//自动追踪模式发送的数据
		{
			sprintf_s(buff, "%d %d %d %d %d %d %d %d %d %d %d %d \r\n", Mode, valid, camX, camY, palmZ,0, 0, 0, 0, 0, 0, 0);
			cout << buff << endl;
		}
		
		//cout << buff << endl;
		

	}
	else//手为空的时候
	{
		
		//sprintf_s(buff, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d \r\n",0,0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0);
		//cout << buff << endl;
	}

}
