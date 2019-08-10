#pragma once

#include <vector>
#include "moveRobot.h"

Position cellShift(MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, Position cur, Position goal, bool derevo);
Position goTo(std::vector<std::pair<int, int>> & points, Position cur, double theta, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2);