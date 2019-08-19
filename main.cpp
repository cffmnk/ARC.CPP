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
#include "slam.h"
#include "moveRobot.h"
#include "path.h"
#include "QR.h"
#include "goTo.h"
#include "alignment.h"
#include "takeCube.h"
#include "ColorDetection.h"
#include "localization.h"
#include "taskMain.h"
#include "taskFinal.h"

const int N = 46;
std::vector<std::vector<int16_t>> field(N, std::vector<int16_t>(N));

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
	Dio_WriteBit(&LED1, 0);
	
	//while (!(bool)Dio_ReadBit(&ButtonL)) {}
	
	mc1.resetEncoders();
	mc2.resetEncoders();
	
	Position pos(0, 0, 0);

	//taskFinal(i2c, mc1, mc2, s1, cap);
	/*
	char x = 'n';
	char y = 'h';
	Position pos((x - 'a') * 115, ('u' - y) * 115, -M_PI / 2);
	vector<vector<int16_t>> f(23, vector<int16_t>(23));
	fill(f);
	f.at(round((pos.y) / 115) + 1).at(round(pos.x / 115) + 1) = 2;
	Lidar lidar;
	grid(&lidar, &f, &pos);
	print_map(f);
	//taskMain(i2c, mc1, mc2, s1, cap, field);
	*/
	{
		int n = 46;
		int cnt = 1;
		Position pos(23 * 115, 23 * 115, 0);
		//vector<vector<int16_t>> field(46, vector<int16_t>(46));
		//fill(f);
		field.at(23 + 1).at(23 + 1) = 2;
		Lidar lidar;
		grid(&lidar, &field, &pos);
		s1.openLeft();
		s1.openRight();
		
		std::vector<Dot> cubes = cubesCoordinates(&field, &pos);
		field.at(23 + 1).at(23 + 1) = 2;
		print_map(field);
		for (int i = 0; i < cubes.size(); ++i)
		{
			std::cout << cubes[i].x <<  " " << cubes[i].y << " " << cubes[i].theta << std::endl;
		}
		cout << "\n";
		
		pii current = pii(23, 23);
		
		int cube_color[10];
		int object_color[10];
	
		for (int i = 0; i < 10; ++i)
		{
			cube_color[i] = -1;
			object_color[i] = -1;
		}
		int needed = 2;
		for (int k = 0; k < cubes.size();++k)
		{
			pii start = current;
			pii goal = current;
			int idx = 0;
			for (int i = k; i < cubes.size(); ++i)
			{
				if (hypotl(current.first - cubes[i].x, current.second - cubes[i].y) < hypotl(current.first - cubes[0].x, current.second - cubes[0].y))
					std::swap(cubes[k], cubes[i]);
			}
		
			if (cubes.size())
				goal = pii(cubes[k].x, cubes[k].y);
			std::cout << current.first << " " << current.second << "\n";
			std::cout << goal.first << " " << goal.second << "\n";
		
			std::vector<pii> points = aStar(current, goal, field);       // build the path
		
			std::cout << "path: " << 0 << "\n";
			for (int i = 0; i < points.size(); ++i)
				std::cout << (int)points[i].first << " " << (int)points[i].second << '\n';
			std::cout << "\n";
		
			pos = goTo(points, pos, cubes[k].theta, &i2c, mc1, mc2); 
			
			delay(1000);
			current = goal;
			
			//shtuka(&i2c, mc1, mc2, &lidar); 
	
			pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, true, true);      // motors reset
			pos = Position(cubes[k].x * 115, cubes[k].y * 115, cubes[k].theta);
			
			if (cube_color[k] < 1)
				cube_color[k] = checkCube(&cap);
			
			if (object_color[k] < 1)
				object_color[k] = checkObject(&cap);
			
			std::cout << "\n";
			std::cout << "colors : " << k << " | " << cube_color[k] << " " << object_color[k] << "\n";
			std::cout << "\n";
			
			if (cube_color[k] == needed)
			{
				bool left = ((cnt == 1) || (cnt == 3));
				bool change = (cnt > 1);
		
				pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, true, true); 
				pos = takeCube(pos, &i2c, mc1, mc2, s1, left, change);   
				++cnt;
				needed = object_color[k];
				int idx = k;
				for (int j = 0; j < k; ++j)
				{
					if (cube_color[j] == needed)
					{
						idx = j - 1;
					}
				}
				k = idx;
			}
			
			delay(2000);
			
		}
		
		delay(1000);
		grid(&lidar, &field, &pos);
		std::vector<Dot> cubes2 = cubesCoordinates(&field, &pos);
		for (int t = 0; t < cubes2.size(); ++t)
		{
			cubes.push_back(cubes2[t]);
		}
	}
	
	
	
	
	
	//task one
	//taskOne(pos, mc1, mc2, s1, i2c, LED1, ButtonL, ButtonR);
	
//	taskTwo(cap);
	//s1.openRight();
	//taskThree(cap);
	//alignment(&i2c, mc1, mc2);
	
	//taskMain(i2c, mc1, mc2, s1, cap, field);
	
	//std::vector<Position> ptr = localization(i2c, mc1, mc2, LED1);
	
	mc1.reset();
	mc2.reset();
	s1.reset();
	
	MyRio_Close();
	return status;
}