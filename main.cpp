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

#include "CoreSLAM.h"

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
	ServoController s1(&i2cA, 3);
	
	Lidar lidar;
	lidar.poll();
	
	
	
	MyRio_Close();
	
	for (int i = 0; i < lidar.ranges.size(); ++i)
		std::cout << lidar.ranges[i] << "\n";
	
	return status;
}
