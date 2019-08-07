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

<<<<<<< HEAD
#include "lidar.h"
#include "initialization.h"
#include "Config.h"
=======
using namespace cv;
using namespace std;

#include "moveRobot.h"
#include "path.h"
#include "QR.h"
#include "goTo.h"
#include "alignment.h"
>>>>>>> parent of 3241663... ,/.,/.

const int N = 23;
std::vector<std::vector<int16_t>> field(N, std::vector<int16_t>(N));

<<<<<<< HEAD

using namespace cv;
using namespace std;
=======
// cam light port 29
>>>>>>> parent of 3241663... ,/.,/.

int main()
{
	NiFpga_Status status;
	MyRio_I2c i2cA;
	NiFpga_Bool buttonState;
	MyRio_Dio Button;
	
	initHardware(&status, &i2cA, &Button);
	
<<<<<<< HEAD
//	  MotorController mc1(&i2cA, 1);
//	  MotorController mc2(&i2cA, 2);
	//  ServoController s1(&i2cA, 3);

	Lidar lidar;
//	delay(1000);
	lidar.poll();
	slam(&lidar);

=======
	MotorController mc1(&i2cA, 1);
	MotorController mc2(&i2cA, 2);
	ServoController s1(&i2cA, 3);
	Lidar l1;
	
	
	mc1.resetEncoders();
	mc2.resetEncoders();
	s1.setSpeed(70, 60, 60, 70, 0, 0);
	
	s1.closeLeft();
	s1.closeRight();
	s1.down();
	std::cout << mc1.batteryVoltage() << std::endl;
	delay(2000);
	
	Position pos(0, 0, 0);
	alignment(&i2cA, l1, mc1, mc2);
	
	return 0;
	
	
	
	
	pos = moveShift(pos, &i2cA, mc1, mc2, 0, -400, 250, 20);
	
	std::vector<Dot> dots = QR(pos, field);
	
	
	cout << "\n";
	for (int i = 0; i < 4; ++i)
	{
		std::cout << dots[i].x <<  " " << dots[i].y << " " << dots[i].theta << std::endl;
	}
	cout << "\n";
	
	print_map(field);
	for (int k = 1; k < 4; ++k)
	{
		pii start(dots[k - 1].x, dots[k - 1].y);
		pii goal(dots[k].x, dots[k].y);
		std::cout << '\n';
		std::cout << start.first << " " << start.second << "\n";
		std::cout << goal.first << " " << goal.second << "\n";
		std::cout << '\n';
		std::vector<pii> points = aStar(start, goal, field);
		std::cout << "path: \n";
		for (int i = 0; i < points.size(); ++i)
			std::cout << (int)points[i].first << " " << (int)points[i].second << '\n';
		
		
		pos = goTo(points, pos, dots[k].theta, &i2cA, mc1, mc2);
		
		delay(2000);
	}
	//delay(1000);	
	
	
	mc1.reset();
	mc2.reset();
	s1.reset();
	
>>>>>>> parent of 3241663... ,/.,/.
	MyRio_Close();
	return status;
}
