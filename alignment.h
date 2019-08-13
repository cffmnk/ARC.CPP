#pragma once

#include "lidar.h"
#include "motors.h"
#include "moveRobot.h"
#include <iostream>
#include <algorithm>
#include <thread> 

void lidarRread(Lidar & l1);

void alignment(MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2);