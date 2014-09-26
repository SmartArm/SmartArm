/******************************************************************************\
* Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/
#include <windows.h>//如果嫌刷屏太快打开这个定义1（一共2处）

#include "Leap.h"
#include<sstream>
#include <stdio.h>
#include<iostream>

#pragma comment(lib,"WS2_32.lib")
using namespace Leap;
using namespace std;
char buff[50];

void getData(const Controller& controller);
void GetHandDirection(const Controller& controller);
void getfingerposition(const Controller& controller);
int main() {

	Controller controller;

	WSADATA wsd;
    SOCKET sockClient;                                            //客户端socket
    SOCKADDR_IN addrSrv;


    sockClient=socket(AF_INET,SOCK_STREAM,0);                    //创建socket
    addrSrv.sin_addr.S_un.S_addr=inet_addr("192.168.1.1");
    addrSrv.sin_family=AF_INET;
    addrSrv.sin_port=htons(8080);
    connect(sockClient,(SOCKADDR*)&addrSrv,sizeof(SOCKADDR));    //连接服务器端

	while(1)
	{
		getData(controller);
		send(sockClient, buff, sizeof(buff), 0);
		Sleep(30);

	}

    closesocket(sockClient);                                    //关闭连接
    WSACleanup();





	return 0;
}

void getData(const Controller& controller)
{
	controller.enableGesture(Gesture::TYPE_CIRCLE);//打开圆圈手势
	//controller.enableGesture(Gesture::TYPE_KEY_TAP);
	//controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
	//controller.enableGesture(Gesture::TYPE_SWIPE);

	const Frame frame = controller.frame();
	if (!frame.hands().isEmpty()) {
		Hand rightHand,leftHand;
		for (int x = 0; x < frame.hands().count(); x++)
		{
			if (frame.hands()[x].isRight())
			{
				rightHand = frame.hands()[x];
			}//获取右手
			else
			{
				leftHand = frame.hands()[x];
			}//获取左手
		}
		
		const Vector direction = rightHand.direction();//手掌方向
		const Vector palmPos = rightHand.palmPosition();//手掌位置
		int palmX, palmY, palmZ;
		palmX = int(palmPos.x);
		palmY = int(palmPos.y);
		palmZ = int(palmPos.z);


		int pitch_int,roll_int,yaw_int;
		pitch_int = int(direction.pitch() * RAD_TO_DEG);
		roll_int = int(rightHand.palmNormal().roll() * RAD_TO_DEG);
		yaw_int = int(direction.yaw() * RAD_TO_DEG);
		// Calculate the hand's pitch, roll, and yaw angles


	
		const FingerList fingers = rightHand.fingers();
		int distance;
		if (!fingers.isEmpty()) {
			// 计算拇指和食指之间的距离
			Vector thumbPos, indexFingerPos;
			
			thumbPos = fingers[0].tipPosition();
			indexFingerPos = fingers[1].tipPosition();
			distance = int(thumbPos.distanceTo(indexFingerPos));

		}

		const GestureList gestures = frame.gestures();
		int progress=0;
		for (int g = 0; g < gestures.count(); ++g) {
			Gesture gesture = gestures[g];

			if (gesture.type() == Gesture::TYPE_CIRCLE) {

				CircleGesture circle = gesture;

				if (circle.state() != Gesture::STATE_START) {
					cout << "当前转了" << circle.progress() << "圈" << endl;

				}
				 progress = int(circle.progress());

			}
		}
		sprintf_s(buff, "%d %d %d %d %d %d %d %d \r\n", palmX, palmY, palmZ, pitch_int, roll_int, yaw_int, distance, progress);
		cout << buff << endl;
	}

}