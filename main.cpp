#include <thread>
#include <chrono>
#include <time.h>
#include <string>
#include <iostream>

//#include "lidar.h"
//#include "initialization.h"
//#include "moveRobot.cpp"

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


	//delay(1000);
	
	
	Position pos(0, 0, 0);
	
	pos = moveShift(pos, &i2cA, mc1, mc2, 200, 400, 200, 20);
	
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
	
	
	
	//pos = moveShift(pos, &i2cA, mc1, mc2, 400, 0, 250, 20);
	
	
    //mc1.reset();
	//mc2.reset();
	
	
	
	s1.reset();
	MyRio_Close();
	return status;
}
