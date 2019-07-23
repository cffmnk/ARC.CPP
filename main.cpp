#include <thread>
#include <chrono>
#include <time.h>
#include <string>
#include <iostream>



#include "MyRio_lib/MyRio.h"
#include "MyRio_lib/I2C.h"
#include "MyRio_lib/DIO.h"
#include <opencv2/opencv.hpp>

#include "lidar_driver.h"
#include "initialization.cpp"
#include "motorcontrollers.cpp"
#include "Config.h"
#include "servo.cpp"

//using namespace cv;



int main()
{
	NiFpga_Status status;
	MyRio_I2c i2cA;
	NiFpga_Bool buttonState;
	MyRio_Dio Button;
	
	initHardware(&status, &i2cA, &Button);
	
	auto t1 = std::chrono::high_resolution_clock::now();
	
	MotorController mc1(&i2cA, 1);
	MotorController mc2(&i2cA, 2);
	
	ServoController s1(&i2cA, 3);
	

	
	s1.setSpeed(80, 50, 50, 80);
	
	
	
	s1.down();
	s1.openLeft();
	s1.openRight();
	delay(500);
	
	
	s1.up();
	
	delay(1000);
	s1.closeRight();
	delay(1000);
	
	s1.down();
	
	
	
	
	
	
	
	delay(1000);
	s1.reset();
	


	//cv ::VideoCapture cap(0);
	//cv :: Mat frames;
	//cap >> frames;
	//imshow("lol", frames);
	//waitKey(0);
	//destroyAllWindows();
	
	MyRio_Close();
	return status;
}
