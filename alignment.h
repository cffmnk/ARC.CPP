#pragma once

#include "lidar.h"
#include "motors.h"
#include "moveRobot.h"
#include <iostream>
#include <algorithm>
#include <thread> 

void lidarRread(Lidar & l1);

void toWall(double dis, double precition, int idx, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2);

void alignment(MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2);