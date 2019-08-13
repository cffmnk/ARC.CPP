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
//#include "slam.h"
//
//#include "MyRio_lib/MyRio.h"
//#include "MyRio_lib/I2C.h"
//#include "MyRio_lib/DIO.h"
//#include "MyRio_lib/PWM.h"
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
//int main()
//{
//	//	NiFpga_Status status;
//	//	MyRio_I2c i2cA;
//	//	NiFpga_Bool buttonState;
//	//	MyRio_Dio Button;
//	//		
//	//	initHardware(&status, &i2cA, &Button);
//	//		
//	//	MotorController mc1(&i2cA, 1);
//	//	MotorController mc2(&i2cA, 2);
//	//	ServoController s1(&i2cA, 3);
//	//	
//	//	mc1.resetEncoders();
//	//	mc2.resetEncoders();
//	//	s1.setSpeed(70, 60, 60, 70, 0, 0);
//	//	
//	//	s1.closeLeft();
//	//	s1.closeRight();
//	//	s1.down();
//	//	std::cout << mc1.batteryVoltage() << std::endl;
//	//	
//	////	s1.openLeft();
//	////	s1.openRight();
//	//	while (!Dio_ReadBit(&Button))
//	//{
//	//	}
//	//	delay(1000);
//	//	
//	//	Position pos(0, 0, 0);
//	//	Lidar lidar;
//	//	pos = moveShift(pos, &i2cA, mc1, mc2, 0, -400, 250, 20);
//	//	Position ST = pos;
//	//	lidar.poll();
//	//	
//	//	while (true)
//	//	{
//	//		lidar.poll();
//	//		pos = moveShift(pos, &i2cA, mc1, mc2, 100, 0, 250, 20);
//	//		pos = moveShift(pos, &i2cA, mc1, mc2, 0, -100, 250, 20);
//	//		pos = moveShift(pos, &i2cA, mc1, mc2, -100, 0, 250, 20);
//	//		pos = moveShift(pos, &i2cA, mc1, mc2, 0, 100, 250, 20);
//	//	}
//	//	cout << pos.x << " " << pos.y << "\n";
//	
//	Lidar lidar;
//	lidar.poll();
//	
//	slam(&lidar);
//}