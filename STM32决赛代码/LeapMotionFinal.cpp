
#include <windows.h>//�����ˢ��̫����������1��һ��2����
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
bool TAP = false;//�������������ж�tap�����Ƿ񱻴���
void getData(const Controller& controller);
void GetHandDirection(const Controller& controller);
void getfingerposition(const Controller& controller);
float indexFinPosX, indexFinPosY;
Leap::Vector startPoint, endPoint;
bool clenchFist = false;//�����Ƿ���ȭ�ı�־
int selecting = 0;//��־��ѡ״̬��0�������ֶ�����ģʽ�£�1����ʼ��ѡ��2�����ѡ�У�3�����ѡ��
clock_t startTime, endTime;
int Mode = 1;//ģʽѡ��
int normalPause = 0;//�ֶ�ģʽ�µ���ͣ
int TrackingPause = 0;//׷��ģʽ�µ���ͣ
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
	SOCKET sockClient;                                            //�ͻ���socket
	SOCKADDR_IN addrSrv;


	sockClient = socket(AF_INET, SOCK_STREAM, 0);                    //����socket
	addrSrv.sin_addr.S_un.S_addr = inet_addr("192.168.1.1");
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(8080);
	connect(sockClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));    //���ӷ�������

	while (1)
	{
		getData(controller);
		if (TAP)
		{
			for (int i = 0; i < 3; i++)
			{
				send(sockClient, buff, sizeof(buff), 0);
				Sleep(10);//��������
			}
			TAP = false;
			tapOrNot = 0;
		}
		send(sockClient, buff, sizeof(buff), 0);
		//system("pause");
		Sleep(10);
	}

	closesocket(sockClient);                                    //�ر�����
	WSACleanup();

	return 0;
}
int valid = 0;
void getData(const Controller& controller)
{
	controller.enableGesture(Gesture::TYPE_CIRCLE);//��ԲȦ����
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
			}//��ȡ����
			else
			{
				leftHand = frame.hands()[x];//
				
			}//��ȡ����
		}
		//*****************************************************����������ݼ��*************************************************************//
		if (leftHand .isValid())
		{

			const Vector palmPosL = leftHand.palmPosition();//��������λ��
			const FingerList leftFingers = leftHand.fingers();//��ȡ������ָ�б�
			int distance1 = 0;
			int nOfFingers1 = leftFingers.count();
			for (int i = 0; i < nOfFingers1; i++)
			{
				distance1 += leftFingers[i].tipPosition().distanceTo(palmPosL);
			}
			distance1 = (int)(distance1 / nOfFingers1);//�������ָ�⵽���ĵ�ƽ������
			//cout << distance1 << endl;
			if ((clenchFist == false) && (distance1 < 70))//��ȭ
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
				cout << "��ȭʱ�䳤��Ϊ��" << duration << endl;
				if (duration >= 1000){
					if (Mode == 1){
						Mode = 2;
						selecting = 1;
						cout << "�л����Զ�׷��ģʽ" << endl;
						cout << "��ʼ��ѡ" << endl;
					}
					else if (Mode == 2)
					{
						Mode = 1;
						cout << "�л����ֶ�����ģʽ" << endl;
					}
				   
				}
				else cout << "ʱ�䲻����" << endl;
				system("pause");
			}
		}
		//*****************************************************����������ݼ��*************************************************************//
		if (rightHand.isValid())
		{

			const Vector direction = rightHand.direction();//���Ʒ���
			const Vector wristPos = rightHand.wristPosition();//����λ��
			const Vector palmnormal = rightHand.palmNormal();//���Ʒ�����
			const Vector palmPos = rightHand.palmPosition();//��������λ��
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
				// ��ȡ����ʳָ��ָ��λ��

				Vector indexFingerPos;
				indexFingerPos = fingers[1].tipPosition();
				indexFinPosX = indexFingerPos.x;
				indexFinPosY = indexFingerPos.y;


				
				int nOfFingers = fingers.count();
				for (int i = 0; i < nOfFingers; i++)
				{
					holdDis += fingers[i].tipPosition().distanceTo(palmPos);
				}
				holdDis = (int)(holdDis / nOfFingers);//�������ָ�⵽���ĵ�ƽ�����룬�ж��Ƿ��צ�ӵ�����
				//////////////////////////////////////////////////////////////////////////
				//���´��������Կ�ѡ׷�ٶ����
				int pinch = 0;
				Vector thumbPos = fingers[0].tipPosition();
				pinch = indexFingerPos.distanceTo(thumbPos);//����Ĵָ��ʳָ�ľ���
				//cout << pinch << endl;
				if (pinch < 50 && selecting == 1)
				{
					startPoint = indexFingerPos;
					cout << "��ʼ��ѡ�����Ϊ" << startPoint.x << "," << startPoint.y << endl;
					selecting = 2;
					system("pause");
				}
				else if (pinch>70 && selecting == 2)
				{
					endPoint = indexFingerPos;
					cout << "������ѡ���յ�Ϊ" << endPoint.x << "," << endPoint.y << endl;
					selecting = 3;
					system("pause");
				}
			}


		}
		//*****************************************************************���������Ʋ���**********************************************************//
		const GestureList gestures = frame.gestures();

		for (int g = 0; g < gestures.count(); ++g) {
			Gesture gesture = gestures[g];

			if (gesture.type() == Gesture::TYPE_CIRCLE) {

				CircleGesture circle = gesture;

				if (circle.state() != Gesture::STATE_START) {
					//cout << "��ǰת��" << circle.progress() << "Ȧ" << endl;

				}
				circleCount = int(circle.progress());
			}
			if (gesture.type() == Gesture::TYPE_KEY_TAP) {

				KeyTapGesture tap = gesture;
				tapOrNot = (int)tap.progress();
				cout << "tap״̬��" << tapOrNot << endl;
				TAP = true;
				
			}
			if (gesture.type() == Gesture::TYPE_SWIPE){
				SwipeGesture swg = gesture;
				HandList handsForGesture = gesture.hands();
				if (handsForGesture.count() == 1 && handsForGesture[0].isLeft())//ֻ�����������������
				{
					if (Mode == 1)//���ֶ�����ģʽ�����ֻӶ���ζ���л��ֶ���ͣ״̬
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
					else if (Mode == 2)//���Զ�׷��ģʽ�»Ӷ�������ζ�����¿�ѡ׷����
					{
						selecting = 1;//���¿�ѡ
						Sleep(100);
						cout << "���¿�ѡ" << endl;
						system("pause");
					}
					
				}
				else if (handsForGesture.count() == 1 && handsForGesture[0].isRight())//ֻ�����������������
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
				case 0:break;//����Ҫ��ѡ
				case 1://��ʼ��ѡ
					
					startPoint = stg.position();//�洢��ʼ��
					cout << "��ʼ��ѡ�����Ϊ" << startPoint.x << "," << startPoint.y<<endl;
					selecting = 2;
					Sleep(100);
					system("pause");
					break;
				case 2://ѡ�������
					endPoint = stg.position();//�洢������
					cout << "������ѡ���յ�Ϊ" << endPoint.x << "," << endPoint.y<<endl;
					selecting = 0;
					Sleep(100);
					system("pause");
					break;
				}
				
				//cout << "��⵽SCREENTAP���ƣ�����Ϊ"<<stg.position().x << " " << stg.position().y<<endl;
				
			}*/
		}
		//sprintf_s(buff, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d \r\n", valid, theta1, theta2, theta3, pitch_int, roll_int, yaw_int, holdDis, progress, tapOrNot, speed, normalPause, Tracking, camX, camY);
		if (Mode == 1)//�ֶ�����ģʽ���͵�����
		{
			sprintf_s(buff, "%d %d %d %d %d %d %d %d %d %d %d %d \r\n", Mode, valid, palmX, palmY, palmZ, pitch_int, roll_int, yaw_int, holdDis, tapOrNot, circleCount, speed);
			cout << buff << endl;
		}
		else if (Mode == 2)//�Զ�׷��ģʽ���͵�����
		{
			sprintf_s(buff, "%d %d %d %d %d %d %d %d %d %d %d %d \r\n", Mode, valid, camX, camY, palmZ,0, 0, 0, 0, 0, 0, 0);
			cout << buff << endl;
		}
		
		//cout << buff << endl;
		

	}
	else//��Ϊ�յ�ʱ��
	{
		
		//sprintf_s(buff, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d \r\n",0,0, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0);
		//cout << buff << endl;
	}

}
