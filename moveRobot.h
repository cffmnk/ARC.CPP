#pragma once
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
#include "motors.h"
#include "Config.h"
#include "servo.h"

const double ROBOT_RADIUS = 147;
const double WHEEL_RADIUS = 50.8;
const double CONSTANT = 1440;

class Pair
{
private:

public:
	double x;
	double y;
	Pair(double, double);
};

class Position
{
private:
    
public:
	double x;
	double y;
	double theta;
	Position(double, double, double);
};

Pair rotate(double, double, double);

Pair shift(double, double, double, double);

Position moveRobot(Position, MyRio_I2c*, MotorController &, MotorController &, double, double, double , bool, bool);

Position moveShift(Position &, MyRio_I2c* , MotorController &, MotorController &, double, double, double, double);
