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
	std::cout << mc1.batteryVoltage() << std::endl;
	delay(2000);
	
	Position pos(0, 0, 0);
	pos = moveShift(pos, &i2cA, mc1, mc2, 0, -400, 250, 20);
	
	s1.openLeft();
	s1.openRight();
	
	std::vector<Dot> dots = QR(pos, field);
	
	Position ST = pos;
	
	dots.push_back(dots[0]);
	dots[4].theta = dots[3].theta;
	
	cout << "\n";
	for (int i = 0; i < 5; ++i)
	{
		std::cout << dots[i].x <<  " " << dots[i].y << " " << dots[i].theta << std::endl;
	}
	cout << "\n";
	
	print_map(field);
	
	pii current = pii(dots[0].x, dots[0].y);
	
	int cube_color[6];
	int object_color[6];
	
	for (int i = 0; i < 6; ++i)
	{
		cube_color[i] = -1;
		object_color[i] = -1;
	}
	int k = 1;
	
	int needed = -1;//2;
	
	while (k < 5)
	{
		pii start = current; // start point
		pii goal(dots[k].x, dots[k].y); // finish point
		std::vector<pii> points = aStar(start, goal, field); // build the path
		
		std::cout << "path: " << k << "\n";
		for(int i = 0; i < points.size() ; ++i)
			std::cout << (int)points[i].first << " " << (int)points[i].second << '\n';
		std::cout << "\n";
		
		pos = goTo(points, pos, dots[k].theta, &i2cA, mc1, mc2); // get to the point
		
		delay(1000);
		
		if (k == 4) // last point (finish)
		{
			++k;
			continue;
		}
		
		Lidar l1;
		alignment(&i2cA, l1, mc1, mc2); 
		pos = moveRobot(pos, &i2cA, mc1, mc2, 0, 0, 0, true, true); // motors reset
		pos = Position(dots[k].x * 115, dots[k].y * 115, dots[k].theta); // reset position
		
		//cube_color[k] = ;
		//object_color[k] = ;
		
		if (cube_color[k] != needed)
		{
			if (object_color[k + 1] < 0)
			{
				std::swap(dots[k], dots[k + 1]);
				std::swap(object_color[k], object_color[k + 1]);
				std::swap(cube_color[k], cube_color[k + 1]);
			}
			else
			{
				std::swap(dots[k], dots[k + 2]);
				std::swap(object_color[k], object_color[k + 2]);
				std::swap(cube_color[k], cube_color[k + 2]);
			}
			continue;
		}
		else
		{
			needed = object_color[k];
			if (cube_color[k + 1] > 0 && cube_color[k + 1] != needed)
			{
				std::swap(dots[k + 1], dots[k + 2]);
				std::swap(object_color[k + 1], object_color[k + 2]);
				std::swap(cube_color[k + 1], cube_color[k + 2]);
			}
		}
		
		 
		bool left = (k == 2);
		bool change = (k > 1);
		
		pos = takeCube(pos, &i2cA, mc1, mc2, s1, left, change); // collect object
		
		current = goal;
		++k;
		
		delay(2000);
	}
	cout << pos.x << " " << pos.y << "\n";
	cout << ST.x << " " << ST.y << "\n";
	
	pos = cellShift(&i2cA, mc1, mc2, pos, ST, true);
	
	s1.closeLeft();
	s1.closeRight();
	
	pos = moveRobot(pos, &i2cA, mc1, mc2, 0, 0, 0, true, true);  // motors reset
	pos = moveShift(pos, &i2cA, mc1, mc2, 0, 400, 200, 10);
	
	
	
	
	
	
	mc1.reset();
	mc2.reset();
	s1.reset();
	
	MyRio_Close();
	return status;
}
