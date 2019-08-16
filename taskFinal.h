#pragma once

#include <thread>
#include <chrono>
#include <time.h>
#include <string>
#include <iostream>

#include "lidar.h"
#include "initialization.h"
#include "motors.h"
#include "Config.h"
#include "servo.h"

#include "MyRio_lib/MyRio.h"
#include "MyRio_lib/I2C.h"
#include "MyRio_lib/DIO.h"
#include "MyRio_lib/AIO.h"
#include <opencv2/opencv.hpp>
#include "VL53L0X.h"

using namespace cv;
using namespace std;

#include "moveRobot.h"
#include "path.h"
#include "QR.h"
#include "goTo.h"
#include "alignment.h"
#include "takeCube.h"
#include "ColorDetection.h"
#include "localization.h"
#include "taskMain.h"
#include <mutex>

void taskFinal(MyRio_I2c & i2c, MotorController & mc1, MotorController & mc2, ServoController & s1, cv::VideoCapture & cap);

