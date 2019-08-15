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

#include "moveRobot.h"
#include "path.h"
#include "QR.h"
#include "goTo.h"
#include "alignment.h"
#include "takeCube.h"
#include "ColorDetection.h"
#include "localization.h"
#include "taskMain.h"

const int N = 23;
std::vector<std::vector<int16_t>> field(N, std::vector<int16_t>(N));

// cam light port 29

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

double sharpRange(MyRio_Aio* sharp)
{
	double volts;
	volts = Aio_Read(sharp);
	return 1 / volts * 27.5;
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
	//	
	//	while (true)
	//	{
	//		ai_A0 = Aio_Read(&A0);
	//		float cm = 1 / ai_A0 * 27.5;
	//		printf("dist = %f\n", cm);
	//		delay(300);
	//	}
	//	
	//	VL53L0X laser(&i2c);
	//	
	//	laser.init();
	//	laser.setTimeout(500);
	//	laser.startContinuous();
	//	
	//	while (true)
	//	{
	//		cout << laser.readRangeContinuousMillimeters() << "\n";
	//	}

	
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
	//
	
		while(!(bool)Dio_ReadBit(&ButtonL)) {}
	
	mc1.resetEncoders();
	mc2.resetEncoders();
	//	
	
		Position pos(0, 0, 0);
	
	//	taskMain(i2c, mc1, mc2, s1, cap, field);
	
	
		//task one
		//taskOne(pos, mc1, mc2, s1, i2c, LED1, ButtonL, ButtonR);
	
		//taskTwo(cap);
		//s1.openRight();
		//taskThree(cap);
		//alignment(&i2c, mc1, mc2);
	
		//taskMain(i2c, mc1, mc2, s1, cap, field);
	
		//std::vector<Position> ptr = localization(i2c, mc1, mc2, LED1);
//
//	
	mc1.reset();
	mc2.reset();
	s1.reset();
	
	MyRio_Close();
	return status;
}
