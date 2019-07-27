#include <thread>
#include <chrono>
#include <time.h>
#include <string>
#include <iostream>

#include "MyRio_lib/MyRio.h"
#include "MyRio_lib/I2C.h"
#include "MyRio_lib/DIO.h"
#include <opencv2/opencv.hpp>

#include "lidar.h"
#include "initialization.h"
#include "motorcontrollers.cpp"
#include "Config.h"
#include "servo.cpp"


using namespace cv;
using namespace std;

int main()
{
	NiFpga_Status status;
	MyRio_I2c i2cA;
	NiFpga_Bool buttonState;
	MyRio_Dio Button;
	
	
	initHardware(&status, &i2cA, &Button);
	
	MotorController mc1(&i2cA, 1);
	MotorController mc2(&i2cA, 2);
//	ServoController s1(&i2cA, 3);
	mc1.enable();
	mc2.enable();
	mc1.resetEncoders();
	mc2.resetEncoders();
	//s1.enable();


	delay(2000);
	

	//
	
	
	
	//Lidar lidar;
	//lidar.poll();
	//std::cout << lidar.ranges[30] << "\n";
	Position pos(0, 0, 0);
	/*
	for (int i = 0; i < 10; ++i)
	{
		pos = moveRobot(pos, &i2cA, mc1, mc2, 200, 200, 0);
		cout << pos.x << " " << pos.y <<  " " << pos.theta << "\n";
	}
	*/
	
	
	pos = moveShift(pos, &i2cA, mc1, mc2, 400, 0, 250, 20);
	
	
	mc1.reset();
	mc2.reset();
	//s1.reset();
	MyRio_Close();
	return status;
}
