#pragma once
#include <iostream>
#include <thread>
#include <chrono>

#include "MyRio_lib/I2C.h"
#include "Config.h"

#define SC_FIRMWARE 0x26
#define SC_SET_ID 0x24
#define SC_VOLTAGE 0x53
#define SC_WDT_STOP 0x23
#define SC_ENABLE 0x25
#define SC_RESET 0x27
#define SC_S1_SPEED 0x28
#define SC_S2_SPEED 0x29
#define SC_S3_SPEED 0x2A
#define SC_S4_SPEED 0x2B
#define SC_S5_SPEED 0x2C
#define SC_S6_SPEED 0x2D
#define SC_S16_SPEED 0x2E
#define SC_S1_POSITION 0x2F
#define SC_S2_POSITION 0x30
#define SC_S3_POSITION 0x31
#define SC_S4_POSITION 0x32
#define SC_S5_POSITION 0x33
#define SC_S6_POSITION 0x34
#define SC_S16_POSITION 0x35
#define SC_CR1 0x36;
#define SC_CR2 0x37;
#define SC_S1_READ 0x38;
#define SC_S2_READ 0x39;
#define SC_S3_READ 0x3A;
#define SC_S4_READ 0x3B;
#define SC_S5_READ 0x3C;
#define SC_S6_READ 0x3D;


/*
 * servo 1 | left hand 
 * open = 70
 * close = 5-10
 *
 * servo 4 | right hand
 * close = 75
 * open = 10
 *
 *
 * servo 2 | neck
 * left
 * 0 behind
 * 
 * servo 3 | neck right
 * zero in front
 * 
 * 0 & 130 same
 * 
 *zero 25 95
 *up 140 0
 *
 **/


class ServoController
{
private:
	MyRio_I2c* i2c_;
	uint8_t address_;

public:

	ServoController(MyRio_I2c*, uint8_t);
    
	~ServoController();
    
	void setAddress(uint8_t);
    
	uint16_t batteryVoltage();
    
	void setWDTReset();
    
	void enable();
    
	void reset();
    
	void setSpeed(uint8_t, uint8_t);
    
	void setSpeed(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
    
	void setPosition(uint8_t, uint8_t);

	void setPositions(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

	uint8_t readPosition(uint8_t);
	
	void openLeft();
	
	void openRight();
	
	void closeLeft();
	
	void closeRight();
	
	void up();
	
	void down();
};