#pragma once

#include <vector>
#include "moveRobot.h"

Position goTo(std::vector<std::pair<int, int>> & points, Position cur, double theta, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2);