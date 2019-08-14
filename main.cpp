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
#include "goTo.h"
#include "alignment.h"
#include "takeCube.h"
#include "ColorDetection.h"
#include "localization.h"

const int N = 23;
std::vector<std::vector<int16_t>> field(N, std::vector<int16_t>(N));

// cam light port 29

int main()
{
	NiFpga_Status status;
	MyRio_I2c i2c;
	NiFpga_Bool buttonState;
	MyRio_Dio Button;
	MyRio_Dio LED1, LED2, LED3;
	MyRio_Dio ButtonL;
	MyRio_Dio ButtonR;
	
	initHardware(&status, &i2c, &ButtonL, &ButtonR, &LED1, &LED2);
	
	MotorController mc1(&i2c, 1);
	MotorController mc2(&i2c, 2);
	ServoController s1(&i2c, 3);
	
	cv::VideoCapture cap(0);
	
	mc1.resetEncoders();
	mc2.resetEncoders();
	s1.setSpeed(70, 60, 60, 70, 0, 0);
	s1.closeLeft();
	s1.closeRight();
	s1.down();
	std::cout << mc1.batteryVoltage() << std::endl;
	delay(2000);
	
	
	
	Position pos(0, 0, 0);
	
	//task one
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
	
	delay(5000);
	
	//alignment(&i2c, mc1, mc2);
	
	//taskMain(i2c, mc1, mc2, s1, cap, field);
	
	//std::vector<Position> ptr = localization(i2c, mc1, mc2, LED1);

	mc1.reset();
	mc2.reset();
	s1.reset();
	
	MyRio_Close();
	return status;
}
