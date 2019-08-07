#pragma once

#include "lidar.h"
#include "motors.h"
#include "moveRobot.h"
#include <iostream>
#include <algorithm>

void alignment(MyRio_I2c* i2c, Lidar & l1, MotorController & mc1, MotorController & mc2);