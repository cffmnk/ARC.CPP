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
#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
	NiFpga_Status status;
	MyRio_I2c i2cA;
	NiFpga_Bool buttonState;
	MyRio_Dio Button;
	
	initHardware(&status, &i2cA, &Button);
	{
		uint8_t w[1] = {MC_ENABLE};
		I2c_Write(&i2cA, 1, w, 1);
		auto t1 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	{
		uint8_t w[1] = {MC_ENABLE};
		I2c_Write(&i2cA, 2, w, 1);
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
//	{
//		int16_t speed = -200;
//		uint8_t w[3] = {MC_M2_SPEED};
//		w[1] = speed >> 8;
//		w[2] = speed & 0xFF;
//		I2c_Write(&i2cA, 1, w, 3);
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
//	{
//		int16_t speed = -200;
//		uint8_t w[3] = {MC_M1_SPEED};
//		w[1] = speed >> 8;
//		w[2] = speed & 0xFF;
//		I2c_Write(&i2cA, 1, w, 3);
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
//	{
//		int16_t speed = 200;
//		uint8_t w[3] = {MC_M1_SPEED};
//		w[1] = speed >> 8;
//		w[2] = speed & 0xFF;
//		I2c_Write(&i2cA, 2, w, 3);
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
//	{
//		int16_t speed = 200;
//		uint8_t w[3] = {MC_M2_SPEED};
//		w[1] = speed >> 8;
//		w[2] = speed & 0xFF;
//		I2c_Write(&i2cA, 2, w, 3);
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
//	
//	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
//	{
//		int16_t speed = 200;
//		uint8_t w[3] = {MC_M2_SPEED};
//		w[1] = speed >> 8;
//		w[2] = speed & 0xFF;
//		I2c_Write(&i2cA, 1, w, 3);
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
	
//	{
//		int16_t speed = 200;
//		uint8_t w[3] = {MC_M1_SPEED};
//		w[1] = speed >> 8;
//		w[2] = speed & 0xFF;
//		I2c_Write(&i2cA, 1, w, 3);
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
	
//	{
//		int16_t speed = -200;
//		uint8_t w[3] = {MC_M1_SPEED};
//		w[1] = speed >> 8;
//		w[2] = speed & 0xFF;
//		I2c_Write(&i2cA, 2, w, 3);
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
//	{
//		int16_t speed = -200;
//		uint8_t w[3] = {MC_M2_SPEED};
//		w[1] = speed >> 8;
//		w[2] = speed & 0xFF;
//		I2c_Write(&i2cA, 2, w, 3);
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
//	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	//	{
	//		int16_t speed = 0;
	//		uint8_t w[3] = {MC_M2_SPEED};
	//		w[1] = speed >> 8;
	//		w[2] = speed & 0xFF;
	//		I2c_Write(&i2cA, 1, w, 3);
	//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	//	}
	//	{
	//		int16_t speed = 0;
	//		uint8_t w[3] = {MC_M1_SPEED};
	//		w[1] = speed >> 8;
	//		w[2] = speed & 0xFF;
	//		I2c_Write(&i2cA, 1, w, 3);
	//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	//	}
	//	{
	//		int16_t speed = 0;
	//		uint8_t w[3] = {MC_M1_SPEED};
	//		w[1] = speed >> 8;
	//		w[2] = speed & 0xFF;
	//		I2c_Write(&i2cA, 2, w, 3);
	//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	//	}
	//	{
	//		int16_t speed = 0;
	//		uint8_t w[3] = {MC_M2_SPEED};
	//		w[1] = speed >> 8;
	//		w[2] = speed & 0xFF;
	//		I2c_Write(&i2cA, 2, w, 3);
	//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	//	}
	
	{
		uint16_t speed1 = -0;
		uint16_t speed2 = -00;
		uint8_t w[5] = {MC_M12_SPEED};
		w[1] = speed1 >> 8;
		w[2] = speed1 & 0xFF;
		w[3] = speed2 >> 8;
		w[4] = speed2 & 0xFF;
		I2c_Write(&i2cA, 1, w, 5);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	{
		uint16_t speed1 = -00;
		uint16_t speed2 = -00;
		uint8_t w[5] = {MC_M12_SPEED};
		w[1] = speed1 >> 8;
		w[2] = speed1 & 0xFF;
		w[3] = speed2 >> 8;
		w[4] = speed2 & 0xFF;
		I2c_Write(&i2cA, 2, w, 5);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	
	VideoCapture cap(0);
	Mat frames;
	cap >> frames;
	imshow("lol", frames);
	waitKey(0);
	destroyAllWindows();
	
	MyRio_Close();
	return status;
}
