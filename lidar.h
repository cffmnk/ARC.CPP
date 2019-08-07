#pragma once

#include <string>
#include <vector>
#include <cmath>

#include "MyRio_lib/UART.h"
#include "Config.h"

#define BaudRate 230400
#define DataBit 8

class Lidar
{
public:
	uint16_t rpms;  // RPMS derived from the rpm bytes in an LFCD packet
	Lidar();
	~Lidar();
	void poll();
	void close() { shutting_down_ = true; }
	std::vector<float> ranges;
	std::vector<std::pair<double, double>> points;
	
private:
	uint32_t baud_rate_;
	bool shutting_down_;
	uint16_t motor_speed_;
	MyRio_Uart uart_;
	int32_t status_ = 0;
};