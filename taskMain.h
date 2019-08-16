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

#include "moveRobot.h"
#include "path.h"
#include "QR.h"
#include "goTo.h"
#include "alignment.h"
#include "takeCube.h"
#include "ColorDetection.h"
#include "localization.h"
#include "slam.h"


void taskMain(MyRio_I2c & i2cA, MotorController & mc1, MotorController & mc2, ServoController & s1, cv::VideoCapture & cap, std::vector<std::vector<int16_t>> field);
