#include <thread>
#include <chrono>
#include <time.h>
#include <string>
#include <iostream>

#include "lidar.h"
#include "initialization.h"
#include "motors.h"
#include "Config.h"
#include "servo.h"

#include "MyRio_lib/MyRio.h"
#include "MyRio_lib/I2C.h"
#include "MyRio_lib/DIO.h"
#include "MyRio_lib/AIO.h"
#include <opencv2/opencv.hpp>
#include "VL53L0X.h"

using namespace cv;
using namespace std;
#include "slam.h"
#include "moveRobot.h"
#include "path.h"
#include "QR.h"
#include "goTo.h"
#include "alignment.h"
#include "takeCube.h"
#include "ColorDetection.h"
#include "localization.h"
#include "taskMain.h"
#include "taskFinal.h"

const int N = 23;
std::vector<std::vector<int16_t>> field(N, std::vector<int16_t>(N));

void taskOne(Position& pos, MotorController& mc1, MotorController& mc2, ServoController& s1, MyRio_I2c& i2c, MyRio_Dio& LED1, MyRio_Dio& ButtonL, MyRio_Dio& ButtonR)
{
	Dio_WriteBit(&LED1, false);
	s1.openLeft();
	s1.openRight();
	pos = takeCube(pos, &i2c, mc1, mc2, s1, false, false);
	
	pos = cellShift(&i2c, mc1, mc2, pos, Position(0, 0, 0), true);
	
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);

	
	
	while (!Dio_ReadBit(&ButtonL) && !Dio_ReadBit(&ButtonR)) {}
	Dio_WriteBit(&LED1, true);
	
	pos = takeCube(pos, &i2c, mc1, mc2, s1, true, true);
	
	delay(8000);
}

void taskTwo(VideoCapture& cap)
{
	Mat frame;
	cap >> frame;
	decode(frame);
}

string colorToText(int num)
{
	switch (num)
	{
	case 1:
		return "orange"; break;
	case 2:
		return "blue";break;
	case 3:
		return "green";break;
	case 4: 
		return "red"; break;
	case 5: 
		return "yellow"; break;
	default:
		return "none"; break;
	}
}

void taskThree(VideoCapture& cap)
{
	cout << "Flower: " << colorToText(checkObject(&cap)) << "\n";
	cout << "Box: " << colorToText(checkCube(&cap)) << "\n";
}



int main()
{
	
	NiFpga_Status status;
	MyRio_I2c i2c;
	NiFpga_Bool buttonState;
	MyRio_Dio Button;
	MyRio_Dio LED1, LED2, LED3;
	MyRio_Dio ButtonL;
	MyRio_Dio ButtonR;

	MyRio_Aio sharpL, sharpR;
	
	sharpR.val = AIA_0VAL;
	sharpR.wght = AIA_0WGHT;
	sharpR.ofst = AIA_0OFST;
	sharpR.is_signed = NiFpga_False;
	Aio_Scaling(&sharpR);
	
	sharpL.val = AIA_1VAL;
	sharpL.wght = AIA_1WGHT;
	sharpL.ofst = AIA_1OFST;
	sharpL.is_signed = NiFpga_False;
	Aio_Scaling(&sharpL);
	
	initHardware(&status, &i2c, &ButtonL, &ButtonR, &LED1, &LED2);
	
	VideoCapture cap(0);
	
	MotorController mc1(&i2c, 1);
	MotorController mc2(&i2c, 2);
	ServoController s1(&i2c, 3);
	
	mc1.resetEncoders();
	mc2.resetEncoders();
	s1.setSpeed(70, 60, 60, 70, 0, 0);
	s1.closeLeft();
	s1.closeRight();
	s1.down();
	std::cout << mc1.batteryVoltage() << std::endl;
	Dio_WriteBit(&LED1, 0);

	
	while (!(bool)Dio_ReadBit(&ButtonL)) {}
	
	mc1.resetEncoders();
	mc2.resetEncoders();
	Position pos(0, 0, 0);

	pos = moveShift(pos, &i2c, mc1, mc2, 0, -450, 250, 20);
	while (true)
	{
		Rect cube(0, 0, 0, 0);
		findCube(&cap, &cube, blue);
		if (cube.x < 200 && cube.x > 60 && cube.width * cube.height > 2100 && cube.y > 40 && cube.y < 130)
		{
			pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, 0, 0);
			break;
		}
		pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0.2, 0, 0);
	}
	
	while (sharpRange(&sharpR) > 20)
	{
		Rect cube;
		findCube(&cap, &cube, blue);
		cout << (cube.x + cube.width) / 2 << " " << (cube.y + cube.height) / 2 << "\n";
		pos = moveRobot(pos, &i2c, mc1, mc2, (cube.x + cube.width) / 2 - 140, 50, 0, 0, 0);
	}
	pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, 0, 0);
	{
		Lidar lidar;
		shtuka(&i2c, mc1, mc2, &lidar, &sharpR, &sharpL);
	}
	s1.openRight();
	auto col = checkObject(&cap);
	takeCube(pos, &i2c, mc1, mc2, s1, false, false);
	
	while (true)
	{
		Rect cube(0, 0, 0, 0);
		findCube(&cap, &cube, col);
		if (cube.x < 200 && cube.x > 60 && cube.width * cube.height > 2100 && cube.y > 40 && cube.y < 130)
		{
			pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, 0, 0);
			break;
		}
		pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0.2, 0, 0);
	}
	
	while (sharpRange(&sharpR) > 20)
	{
		Rect cube;
		findCube(&cap, &cube, col);
		cout << (cube.x + cube.width) / 2 << " " << (cube.y + cube.height) / 2 << "\n";
		pos = moveRobot(pos, &i2c, mc1, mc2, (cube.x + cube.width) / 2 - 140, 50, 0, 0, 0);
	}
	pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, 0, 0);
	{
		Lidar lidar;
		shtuka(&i2c, mc1, mc2, &lidar, &sharpR, &sharpL);
	}
	s1.openLeft();
	takeCube(pos, &i2c, mc1, mc2, s1, true, true);
	s1.closeLeft();
	s1.closeRight();
	col = checkObject(&cap);
	takeCube(pos, &i2c, mc1, mc2, s1, false, false);
	
	while (true)
	{
		Rect cube(0, 0, 0, 0);
		findCube(&cap, &cube, col);
		if (cube.x < 200 && cube.x > 60 && cube.width * cube.height > 2100 && cube.y > 40 && cube.y < 130)
		{
			pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, 0, 0);
			break;
		}
		pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0.2, 0, 0);
	}
	
	while (sharpRange(&sharpR) > 20)
	{
		Rect cube;
		findCube(&cap, &cube, col);
		cout << (cube.x + cube.width) / 2 << " " << (cube.y + cube.height) / 2 << "\n";
		pos = moveRobot(pos, &i2c, mc1, mc2, (cube.x + cube.width) / 2 - 140, 50, 0, 0, 0);
	}
	pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, 0, 0);
	{
		Lidar lidar;
		shtuka(&i2c, mc1, mc2, &lidar, &sharpR, &sharpL);
	}
	s1.openLeft();
	takeCube(pos, &i2c, mc1, mc2, s1, false, true);
	s1.closeLeft();
	s1.closeRight();
//	
//	Position pos(0, 0, 0);
//
//		pos = moveShift(pos, &i2c, mc1, mc2, 0, -450, 250, 20);
//		
//		pos = turn(&i2c, mc1, mc2, pos, Position(pos.x, pos.y, M_PI));
//		
//		while (sharpRange(&sharpL) > 20 || sharpRange(&sharpR) > 20)
//		{
//			pos = moveRobot(pos, &i2c, mc1, mc2, 0, 150, 0, 0, 0);
//		}
//		pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, 0, 0);
//		{
//			Lidar lidar;
//				lidar.poll();
//				int i1 = 0;
//				int i2 = 359;
//			
//				while ((lidar.ranges[i1 + 1] > 12) && (lidar.ranges[i1 + 1] < 40) && i1 < 270)
//					++i1;
//	    
//				while ((lidar.ranges[i2 - 1] > 12) && (lidar.ranges[i2 - 1] < 40) && i2 > 90)
//					--i2;
//			
//				while (((lidar.ranges[i1] <= 12) || (lidar.ranges[i1] >= 40)))
//				{
//					--i1;
//					if (i1 < 0)
//						i1 += 360;
//				}	
//			
//				while (((lidar.ranges[i2] <= 12) || (lidar.ranges[i2] >= 40)))
//				{
//					++i2;
//					if (i2 >= 360)
//						i2 -= 360;
//				}
//		
//				std::pair<float, float> p1 = lidar.points[i1];
//				std::pair<float, float> p2 = lidar.points[i2];
//				std::pair<float, float> pm = {
//					(p1.first + p2.first) / 2.,
//					(p1.second + p2.second) / 2.
//				};
//			
//				double t;
//			
//				double a2_x = lidar.ranges[i2] * cos(i2 * M_PI / 180);
//				double a2_y = lidar.ranges[i2] * sin(i2 * M_PI / 180);
//				double a1_x = lidar.ranges[i1] * cos(i1 * M_PI / 180);
//				double a1_y = lidar.ranges[i1] * sin(i1 * M_PI / 180);
//				double A_X = (a2_x - a1_x);
//				double A_Y = (a2_y - a1_y);
//	        
//				if ((A_X) * (A_X) + (A_Y) * (A_Y) == 0 || std::fabs(((A_X) / sqrt((A_X) * (A_X) + (A_Y) * (A_Y))) > 1))
//					t = 0;
//				else
//					t = std::acos((A_X) / sqrt((A_X) * (A_X) + (A_Y) * (A_Y))) - M_PI / 2 + 0.07;
//		
//				pos = turn(&i2c, mc1, mc2, pos, Position(pos.x, pos.y, pos.theta + t));
//			pos = moveShift(pos, &i2c, mc1, mc2, 0, (pm.first * 10  - 230), 100, 1);
//				//	std::cout << pm.first - 23.7 << " " << pm.second - 1.8 << "\n";
//		}
//	
//	Position realPos(0, 230, 0);
//	cout << pos.y - realPos.y;

	//taskFinal(i2c, mc1, mc2, s1, cap);
	//vector<vector<int>> f(23, vector<int>(23));
	//f.at(round((pos.y) / 115) + 1).at(round(pos.x / 115) + 1) = 2;
	//Lidar lidar;
	//grid(&lidar, &f, &pos);
	
	//task one
	//taskOne(pos, mc1, mc2, s1, i2c, LED1, ButtonL, ButtonR);
	
//	taskTwo(cap);
	//s1.openRight();
	//taskThree(cap);
	//alignment(&i2c, mc1, mc2);
	
	//taskMain(i2c, mc1, mc2, s1, cap, field);
	
	//std::vector<Position> ptr = localization(i2c, mc1, mc2, LED1);
	
	mc1.reset();
	mc2.reset();
	s1.reset();
	
	MyRio_Close();
	return status;
}