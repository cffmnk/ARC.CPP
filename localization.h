#pragma once

#include "moveRobot.h"
#include "path.h"
#include "QR.h"
#include "goTo.h"
#include "alignment.h"
#include "takeCube.h"
#include "ColorDetection.h"

std::vector<Position> localization(MyRio_I2c & i2cA, MotorController & mc1, MotorController & mc2, MyRio_Dio & LED1);
/*
 * 0 - current position
 * 1 - start position
 */
