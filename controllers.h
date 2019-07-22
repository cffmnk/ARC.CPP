#pragma once

#include <stdio.h>

#include "MyRio_lib/MyRio.h"
#include "MyRio_lib/I2C.h"
#include "Config.h"
#include <chrono>
#include <thread>

#define dd(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))

#define MC_FIRMWARE 0x26
#define MC_SET_ID 0x24
#define MC_VOLTAGE 0x53
#define MC_WDT_STOP 0x23
#define MC_ENABLE 0x25
#define MC_RESET 0x27
#define MC_M1_POWER 0x40
#define MC_M2_POWER 0x41
#define MC_M12_POWER 0x42
#define MC_M1_SPEED 0x43
#define MC_M2_SPEED 0x44
#define MC_M12_SPEED 0x45
#define MC_M1_TARGET 0x46
#define MC_M2_TARGET 0x47
#define MC_M12_TARGET 0x48
#define MC_M1_DEGREE 0x58
#define MC_M2_DEGREE 0x59
#define MC_M12_DEGREE 0x5A
#define MC_M1_INVERT 0x51
#define MC_M2_INVERT 0x52
#define MC_M1_BUSY 0x4F
#define MC_M2_BUSY 0x50
#define MC_M1_CURRENT 0x54
#define MC_M2_CURRENT 0x55
#define MC_E1_COUNT 0x49
#define MC_E2_COUNT 0x4A
#define MC_E1_DEGREE 0x5B
#define MC_E2_DEGREE 0x5C
#define MC_E1_RESET 0x4C
#define MC_E2_RESET 0x4D
#define MC_E12_RESET 0x4E
#define MC_SPEED_PID 0x56
#define MC_TARGET_PID 0x57

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

class MotorController
{
private:
	MyRio_I2c* i2c_;
	uint8_t address_;
	
public:
	MotorController(MyRio_I2c* i2c, uint8_t address);
	
	~MotorController();
	
	void enable();
	
	void setInvertState(uint8_t motor, bool state);
	
	uint16_t batteryVoltage();
	
	int32_t readEncoderCount(uint8_t motor);
	
	int32_t readEncoderDegree(uint8_t motor);
	
	uint16_t readCurrent(uint8_t motor);
	
	void reset();
	
	void resetEncoder(uint8_t motor);
	
	void resetEncoders();
	
	void setAddress(uint8_t address);
	
	void setMotorPower(uint8_t motor, int8_t power);
	
	void setMotorsPower(int8_t power1, int8_t power2);
	
	void setMotorSpeed(uint8_t motor, int16_t speed);
	
	void setMotorsSpeed(int16_t speed1, int16_t speed2);
	
//	void setMotorTarget(uint8_t motor, int16_t speed, int32_t target);
//	
//	void setMotorsTarget(int16_t speed1, int32_t target1, int16_t speed2, int32_t target2);
	
	void setWDTReset();
	
	void setSpeedPID(float p, float i, float d);
	
//	void setTargetPID(float p, float i, float d);
};

class ServoController
{
private:
	MyRio_I2c* i2c_;
	uint8_t address_;
	
public:
	ServoController(MyRio_I2c* i2c, uint8_t address);
	
	~ServoController();
	
	void setAddress(uint8_t address);
	
	uint16_t batteryVoltage();
	
	void setWDTReset();
	
	void enable();
	
	void reset();
	
	void setSpeed(uint8_t servo, uint8_t speed);
	
	void setSpeed(uint8_t speed1, uint8_t speed2, uint8_t speed3, uint8_t speed4, uint8_t speed5, uint8_t speed6);
	
	uint8_t getPosition(uint8_t servo);
	
	void getPosition(uint8_t* pos);
};