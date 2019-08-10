//#include <thread>
//#include <chrono>
//#include <time.h>
//#include <string>
//#include <iostream>
//
//#include "lidar.h"
//#include "initialization.h"
//#include "motors.h"
//#include "Config.h"
//#include "servo.h"
//
//#include "MyRio_lib/MyRio.h"
//#include "MyRio_lib/I2C.h"
//#include "MyRio_lib/DIO.h"
//#include <opencv2/opencv.hpp>
//
//using namespace cv;
//using namespace std;
//
//#include "moveRobot.h"
//#include "path.h"
//#include "QR.h"
//#include "goTo.h"
//#include "alignment.h"
//#include "takeCube.h"
//
//const int N = 23;
//std::vector<std::vector<int16_t>> field(N, std::vector<int16_t>(N));
//
//// cam light port 29
//
//int main()
//{
//	NiFpga_Status status;
//	MyRio_I2c i2cA;
//	NiFpga_Bool buttonState;
//	MyRio_Dio Button;
//	
//	initHardware(&status, &i2cA, &Button);
//	
//	MotorController mc1(&i2cA, 1);
//	MotorController mc2(&i2cA, 2);
//	ServoController s1(&i2cA, 3);
//	
//	
//	mc1.resetEncoders();
//	mc2.resetEncoders();
//	s1.setSpeed(70, 60, 60, 70, 0, 0);
//	
//	s1.closeLeft();
//	s1.closeRight();
//	s1.down();
//	std::cout << mc1.batteryVoltage() << std::endl;
//	delay(2000);
//	
//	Position pos(0, 0, 0);
//
//	pos = moveShift(pos, &i2cA, mc1, mc2, 0, -400, 250, 20);
//	
//	s1.openLeft();
//	s1.openRight();
//	
//	std::vector<Dot> dots = QR(pos, field);
//	
//	Position ST = pos;
//	
//	
//	dots.push_back(dots[0]);
//	
//	cout << "\n";
//	for (int i = 0; i < 4; ++i)
//	{
//		std::cout << dots[i].x <<  " " << dots[i].y << " " << dots[i].theta << std::endl;
//	}
//	cout << "\n";
//	
//	print_map(field);
//	
//	for (int k = 1; k < 4; ++k)
//	{
//		pii start(dots[k - 1].x, dots[k - 1].y);
//		pii goal(dots[k].x, dots[k].y);
//		std::vector<pii> points = aStar(start, goal, field);
//		
//		pos = goTo(points, pos, dots[k].theta, &i2cA, mc1, mc2);
//		
//		delay(3000);
//		
//		Lidar l1;
//		
//		alignment(&i2cA, l1, mc1, mc2);
//		moveRobot(pos, &i2cA, mc1, mc2, 0, 0, 0, true, true);
//		
//		//pos = Position(dots[k].x * 115, dots[k].y * 115, dots[k].theta);
//
//		bool left = (k == 2);
//		bool change = (k > 1);
//		
//		pos = takeCube(pos, &i2cA, mc1, mc2, s1, left, change);
//		
//		delay(2000);
//	}
//	
//	
//	
//	
//	mc1.reset();
//	mc2.reset();
//	s1.reset();
//	
//	MyRio_Close();
//	return status;
//}
