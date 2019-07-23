	#pragma once

#include <stdio.h>

#include "MyRio_lib/MyRio.h"
#include "MyRio_lib/I2C.h"
#include "Config.h"
#include <chrono>
#include <thread>

#define delay(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))

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
    MotorController(MyRio_I2c* i2c, uint8_t address)
    {
        i2c_ =  i2c;
        address_ = address;
	    reset();
        enable();
    }
    
    ~MotorController()
    {
	    reset();
        delay(10);
    }
    
    void enable()
    {
        uint8_t writeData = MC_ENABLE;
        I2c_Write(i2c_, address_, &writeData, 1);
        delay(30);
    }
    
    void setInvertState(uint8_t motor, bool state)
	{
		uint8_t writeData[2];
		if (motor == 1)
			writeData[0] = MC_M1_INVERT;
		else if (motor == 2)
			writeData[0] = MC_M2_INVERT;
		else
			return;
	
		writeData[1] = state;
		I2c_Write(i2c_, address_, writeData, 2);
		delay(10);
	}
    
    uint16_t batteryVoltage()
	{
		uint8_t writeData = MC_VOLTAGE;
		I2c_Write(i2c_, address_, &writeData, 1);
		uint8_t readData[2];
		I2c_Read(i2c_, address_, readData, 2);
		delay(10);
		return (readData[0] << 8) + readData[1];
	}
    
    int32_t readEncoderCount(uint8_t motor)
	{
		uint8_t writeData;
		if (motor == 1)
			writeData = MC_E1_COUNT;
		else if (motor == 2)
			writeData = MC_E2_COUNT;
		else
			return 0;
	
		I2c_Write(i2c_, address_, &writeData, 1);
		uint8_t readData[4];
		I2c_Read(i2c_, address_, readData, 4);
		int32_t result = 0;
		for (int i = 0; i < 4; ++i)
			result = (result << 8) + readData[3 - i];
		delay(10);
		return result;
	}
    
    int32_t readEncoderDegree(uint8_t motor)
	{
		uint8_t writeData;
		if (motor == 1)
			writeData = MC_E1_DEGREE;
		else if (motor == 2)
			writeData = MC_E2_DEGREE;
		else
			return 0;
	
		I2c_Write(i2c_, address_, &writeData, 1);
		uint8_t readData[4];
		I2c_Read(i2c_, address_, readData, 4);
		int32_t result = 0;
		for (int i = 0; i < 4; ++i)
			result = (result << 8) + readData[i];
		delay(30);
		return result;
	}
    
    uint16_t readCurrent(uint8_t motor)
	{
		uint8_t writeData;
		if (motor == 1)
			writeData = MC_M1_CURRENT;
		else if (motor == 2)
			writeData = MC_M2_CURRENT;
		else
			return 0;
	
		I2c_Write(i2c_, address_, &writeData, 1);
		uint8_t readData[2];
		I2c_Read(i2c_, address_, readData, 2);
		delay(10);
		return (readData[0] << 8) + readData[1];
	}
    
	void reset()
	{
		uint8_t writeData = MC_RESET;
		I2c_Write(i2c_, address_, &writeData, 1);
		delay(50);
	}
		
    void resetEncoder(uint8_t motor)
	{
		uint8_t writeData;
		if (motor == 1)
			writeData = MC_E1_RESET;
		else if (motor == 2)
			writeData = MC_E2_RESET;
		else
			return;
		I2c_Write(i2c_, address_, &writeData, 1);
		delay(20);
	}
    
    void resetEncoders()
	{
		uint8_t writeData = MC_E12_RESET;
		I2c_Write(i2c_, address_, &writeData, 1);
		delay(20);
	}
    
    void setAddress(uint8_t address)
	{
		uint8_t writeData[2] = { MC_SET_ID, address };
		I2c_Write(i2c_, address_, writeData, 2);
		delay(10);
	}
    
    void setMotorPower(uint8_t motor, int8_t power)
    {
        uint8_t writeData[2];
        if (motor == 1)
            writeData[0] = MC_M1_POWER;
        else if (motor == 2)
            writeData[0] = MC_M2_POWER;
        else
            return;
        
        writeData[1] = power;
        I2c_Write(i2c_, address_, writeData, 2);
	    delay(30);
    }
    
    void setMotorsPower(int8_t power1, int8_t power2)
	{
		uint8_t writeData[3] = { MC_M12_POWER, static_cast<uint8_t>(power1), static_cast<uint8_t>(power2) };
		I2c_Write(i2c_, address_, writeData, 3);
		delay(10);
	}
    
    void setMotorSpeed(uint8_t motor, int16_t speed)
	{
		uint8_t writeData[3];
		if (motor == 1)
			writeData[0] = MC_M1_SPEED;
		else if (motor == 2)
			writeData[0] = MC_M2_SPEED;
		else
			return;
	
		writeData[1] = speed >> 8;
		writeData[2] = speed & 0xFF;
		I2c_Write(i2c_, address_, writeData, 3);
		delay(30);
	}
    
    void setMotorsSpeed(int16_t speed1, int16_t speed2)
	{
		uint8_t writeData[5];
		writeData[0] = MC_M12_SPEED;
		writeData[1] = speed1 >> 8;
		writeData[2] = speed1 & 0xFF;
		writeData[3] = speed2 >> 8;
		writeData[4] = speed2 & 0xFF;
		I2c_Write(i2c_, address_, writeData, 5);
		delay(10);
	}
    
	void setMotorTarget(uint8_t motor, int16_t speed, int32_t target)
	{
		uint8_t writeData[7];
		if (motor == 1)
			writeData[0] = MC_M1_TARGET;
		else if (motor == 2)
			writeData[0] = MC_M2_TARGET;
		writeData[1] = speed >> 8;
		writeData[2] = speed & 0xFF;
		writeData[3] = target >> 24;
		writeData[4] = target >> 16;
		writeData[5] = target >> 8;
		writeData[6] = target & 0xFF;
		I2c_Write(i2c_, address_, writeData, 7);
		delay(20);
	}
//  
	void setMotorsTarget(int16_t speed1, int32_t target1, int16_t speed2, int32_t target2)
	{
		uint8_t writeData[13] = { MC_M12_TARGET };
		writeData[1] = speed1 >> 8;
		writeData[2] = speed1 & 0xFF;
		writeData[3] = target1 >> 24;
		writeData[4] = target1 >> 16;
		writeData[5] = target1 >> 8;
		writeData[6] = target1 & 0xFF;
		writeData[7] = speed2 >> 8;
		writeData[8] = speed2 & 0xFF;
		writeData[9] = target2 >> 24;
		writeData[10] = target2 >> 16;
		writeData[11] = target2 >> 8;
		writeData[12] = target2 & 0xFF;
		I2c_Write(i2c_, address_, writeData, 13);
	}
    
    void setWDTReset()
	{
		uint8_t writeData = MC_WDT_STOP;
		I2c_Write(i2c_, address_, &writeData, 1);
		delay(10);
	}
    
	void setSpeedPID(float p = 1.5, float i = 2.5, float d = 0.008)
	{
		uint8_t writeData[7];
		uint16_t p_ = p * 1000;
		uint16_t i_ = i * 1000;
		uint16_t d_ = d * 1000;
		
		writeData[0] = MC_SPEED_PID;
		writeData[1] = p_ >> 8;
		writeData[2] = p_ & 0xFF;
		writeData[3] = i_ >> 8;
		writeData[4] = i_ & 0xFF;
		writeData[5] = d_ >> 8;
		writeData[6] = d_ & 0xFF;

		I2c_Write(i2c_, address_, writeData, 1);
		delay(10);
	}
	
	bool isBusy(uint8_t motor)
	{
		uint8_t writeData[1];
		if (motor == 1)
			writeData[0] = MC_M1_BUSY;
		else if (motor == 2)
			writeData[0] = MC_M2_BUSY;
		else 
			return false;
		uint8_t res;
		I2c_Write(i2c_, address_, writeData, 1);
		I2c_Read(i2c_, address_, &res, 1);
		delay(10);
		return res;
	}
    
	void setTargetPID(float p = 1.5, float i = 0, float d = 0.005)
	{
		uint8_t writeData[7];
		uint16_t p_ = p * 1000;
		uint16_t i_ = i * 1000;
		uint16_t d_ = d * 1000;
		
		writeData[0] = MC_TARGET_PID;
		writeData[1] = p_ >> 8;
		writeData[2] = p_ & 0xFF;
		writeData[3] = i_ >> 8;
		writeData[4] = i_ & 0xFF;
		writeData[5] = d_ >> 8;
		writeData[6] = d_ & 0xFF;

		I2c_Write(i2c_, address_, writeData, 1);
		delay(10);
	}
};