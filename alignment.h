#pragma once

#include "lidar.h"
#include "motors.h"
#include "moveRobot.h"
#include <iostream>
#include <algorithm>
#include <thread> 
#include "goTo.h"

void lidarRread(Lidar & l1);

void toWall(double dis, double precition, int idx, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, Lidar* lidar);
Position encToWall(Position pos, double dis, double precition, int idx, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, Lidar* lidar);
void center(double precition, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, Lidar* lidar);
void shtuka(MyRio_I2c* i2c, MotorController& mc1, MotorController& mc2, Lidar* lidar);
void alignment(MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, Lidar* lidar);