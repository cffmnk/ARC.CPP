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
//#include "localization.h"

const int N = 23;
std::vector<std::vector<int16_t>> field(N, std::vector<int16_t>(N));

// cam light port 29

int main()
{
	NiFpga_Status status;
	MyRio_I2c i2c;

	initHardware(&i2c);
	
	MotorController mc1(&i2c, 1);
		MotorController mc2(&i2c, 2);
		ServoController s1(&i2c, 3);
	
	s1.closeLeft();
	s1.closeRight();
	s1.down();
	
	
	Position pos(0, 0, 0);
	pos = moveRobot(pos, &i2c, mc1, mc2, 0, 200, 0, 0, 0);
	delay(2000);
	mc1.reset();
	mc2.reset();
	s1.reset();
	
	MyRio_Close();
	return status;
}
