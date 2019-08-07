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

const double ROBOT_RADIUS = 146.3;
const double WHEEL_RADIUS = 50.8;
const double CONSTANT = 1440;

class Position
{
private:
    
public:
	double x;
	double y;
	double theta;
	Position(double, double, double);
};

std::pair<double, double> rotate(double, double, double);

std::pair<double, double> shift(double, double, double, double);

Position moveRobot(Position pos, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, double x, double y, double theta, bool, bool);

Position moveShift(Position & pos, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, double x, double y, double max_speed, double precision);

void move(MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, double x, double y, double theta, bool reset = false);

