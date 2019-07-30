#pragma once

#include <stdio.h>

#include "MyRio_lib/MyRio.h"
#include "MyRio_lib/I2C.h"
#include "Config.h"
#include <chrono>
#include <thread>

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

class MotorController
{
private:
	MyRio_I2c* i2c_;
	uint8_t address_;
    
public:
	MotorController(MyRio_I2c*, uint8_t);
    
	~MotorController();
    
	void enable();
    
	void setInvertState(uint8_t, bool);
    
	uint16_t batteryVoltage();
    
	int32_t readEncoderCount(uint8_t);
    
	int32_t readEncoderDegree(uint8_t);
    
	uint16_t readCurrent(uint8_t);
    
	void reset();
		
	void resetEncoder(uint8_t);
    
	void resetEncoders();
    
	void setAddress(uint8_t);
    
	void setMotorPower(uint8_t, int8_t);
    
	void setMotorsPower(int8_t, int8_t);
    
	void setMotorSpeed(uint8_t, int16_t);
    
	void setMotorsSpeed(int16_t, int16_t);
    
	void setMotorTarget(uint8_t, int16_t, int32_t );
	 
	void setMotorsTarget(int16_t, int32_t, int16_t, int32_t);
    
	void setWDTReset();
    
	void setSpeedPID(float, float, float);
	
	bool isBusy(uint8_t);
    
	void setTargetPID(float, float, float);
};