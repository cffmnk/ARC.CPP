#include <thread>
#include <chrono>
#include <time.h>
#include <string>
#include <iostream>

#include "lidar_driver.h"
#include "initialization.cpp"
#include "controllers.h"
#include "Config.h"

#include "MyRio_lib/MyRio.h"
#include "MyRio_lib/I2C.h"
#include "MyRio_lib/DIO.h"

int main()
{
	NiFpga_Status status;
	MyRio_I2c i2cA;
	NiFpga_Bool buttonState;
	MyRio_Dio Button;
	LFCDLaser lidar;
	MotorController mc_1(&i2cA, 1);
	MotorController mc_2(&i2cA, 2);
	initHardware(&status, &i2cA, &Button);
	
	
	MyRio_Close();
	return status;
}
