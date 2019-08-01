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
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#include "moveRobot.h"
#include "path.h"
#include "QR.h"

const int N = 23;
std::vector<std::vector<int16_t>> field(N, std::vector<int16_t>(N));

// cam light port 29



int main()
{
	NiFpga_Status status;
	MyRio_I2c i2cA;
	NiFpga_Bool buttonState;
	MyRio_Dio Button;
	
	initHardware(&status, &i2cA, &Button);
	
	MotorController mc1(&i2cA, 1);
	MotorController mc2(&i2cA, 2);
	ServoController s1(&i2cA, 3);
	
	mc1.resetEncoders();
	mc2.resetEncoders();
	s1.setSpeed(70, 60, 60, 70, 0, 0);
	
	s1.closeLeft();
	s1.closeRight();
	s1.down();
	delay(2000);
	
	V start(3, 3);
	V goal(10, 7);
	
	
	
	std::vector<PairI> res = aStar(start, goal, field);
	
	Position pos(0, -400, 0);
	
	cv::VideoCapture cap(0);
	cv::Mat im;
	cap >> im;

decode(im);
	//QR(pos, field, qr);
	//pos = moveShift(pos, &i2cA, mc1, mc2, 0, 345, 250, 2);
	//delay(1000);
	//std::cout << "\n";
	//pos = moveShift(pos, &i2cA, mc1, mc2, 230, 0, 250, 2);
	
	
	
	//QR(pos);
	
	/*
	std::cout << "path: \n";
	
	for (int i = 0; i < res.size(); ++i)
	{
		std::cout << (int)res[i].x << " " << (int)res[i].y << '\n';
	}
	*/

	//delay(1000);	
	
	mc1.reset();
	mc2.reset();
	s1.reset();
	
	MyRio_Close();
	return status;
}
