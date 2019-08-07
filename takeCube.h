#pragma once

#include "moveRobot.h"

Position takeCube(Position & cur, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, ServoController & s1, bool leftArm = false, bool change = false);