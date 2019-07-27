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

double prevEncoders[4] = { 0, 0, 0, 0 };

const double ROBOT_RADIUS = 147;
const double WHEEL_RADIUS = 50.8;
const double CONSTANT = 1440;

class Pair
{
private:

public:
	double x;
	double y;
    
	Pair(double x_ = 0, double y_ = 0)
	{
		x = x_;
		y = y_;
	}
};

class Position
{
private:
    
public:
	double x;
	double y;
	double theta;
	Position(double x_ = 0, double y_ = 0, double theta_ = 0)
	{
		x = x_;
		y = y_;
		theta = theta_;
	}
};//

Pair rotate(double x, double y, double theta)
{
	Pair res;
	res.x = x * cos(theta) + y * sin(theta);
	res.y = -x * sin(theta) + y * cos(theta);
	return res;
}

Pair shift(double x, double y, double dx, double dy)
{
	return Pair(x - dx, y - dy);
}


///*
Position moveRobot(Position cur, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, double x, double y, double theta, bool reset = false, bool relativeToFrame = false)
{
	Position pos = Position(x, y, theta);
	if (reset)
	{
		mc1.resetEncoders();
		mc2.resetEncoders();
		for (int i = 0; i < 4; ++i)
			prevEncoders[i] = 0;
		return pos;
	}

	if (relativeToFrame)
	{
		Pair c1 = rotate(pos.x, pos.y, cur.theta);
		pos.x = c1.x;
		pos.y = c1.y;
	}

	double v[4];
	for (int i = 0; i < 4; ++i)
	{
		double alpha = (2 * i + 1) * M_PI / 4;
		v[i] = (-sin(alpha) * pos.x + cos(alpha) * pos.y + ROBOT_RADIUS * pos.theta) / WHEEL_RADIUS;
	}

	for (int i = 0; i < 4; ++i)
	{
		v[i] = v[i] / M_PI * 180;
	}
	//std::cout << "V: " << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << "\n";

    mc1.setMotorsSpeed(v[2], v[1]);
	mc2.setMotorsSpeed(v[0], v[3]);

	double newEncoders[4] = { 0, 0, 0, 0 };
	
	newEncoders[0] = mc2.readEncoderCount(1);
	newEncoders[1] = mc1.readEncoderCount(2);
	newEncoders[2] = mc1.readEncoderCount(1);
	newEncoders[3] = mc2.readEncoderCount(2);

	double dv[4];
	//std::cout << "Enc ";
    for(int i = 0 ; i < 4 ; ++i)
	{
		dv[i] = (2 * M_PI / CONSTANT) * (newEncoders[i] - prevEncoders[i]);
		//std::cout << newEncoders[i] << " ";
		prevEncoders[i] = newEncoders[i];
	    
	}
	//std::cout << "\n";

    pos.theta = WHEEL_RADIUS * (dv[0] + dv[1] + dv[2] + dv[3]) / (4 * ROBOT_RADIUS);
	pos.x = ((dv[3] - dv[0]) + (dv[2] - dv[1])) / 2 * WHEEL_RADIUS / sqrt(2);
	pos.y = ((dv[1] - dv[0]) + (dv[2] - dv[3])) / -2 * WHEEL_RADIUS / sqrt(2);

	Pair c1 = rotate(pos.x, pos.y, -cur.theta);

	pos.x = cur.x + c1.x;
	pos.y = cur.y + c1.y;
	pos.theta += cur.theta;

	return pos;
}
//*/


Position moveShift(Position &cur, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, double x_shift, double y_shift, double max_speed, double precision) 
{
	Position pos = cur;

	Pair c1 = rotate(x_shift, y_shift, -pos.theta);
    
	Pair c2 = shift(cur.x, cur.y, c1.x, c1.y);

	pos = cur;

	double x_speed = 2 * precision;
	double y_speed = 2 * precision;
	
	std::cout << "VFD " << c2.x << " " << c2.y << "\n";
	

	while ((std::abs(x_speed) > precision) || (std::abs(y_speed) > precision))
	{
		x_speed = std::max(std::min((c2.x - pos.x) * 3, max_speed), -max_speed);
		y_speed = std::max(std::min((c2.y - pos.y) * 3, max_speed), -max_speed);
		pos = moveRobot(pos, i2c, mc1, mc2, x_speed, y_speed, 0, false, true);

	}

	//pos = moveRobot(pos, i2c, mc1, mc2, 0, 0, 0, false, true);
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);

	return pos;
}