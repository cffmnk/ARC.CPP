#include "motors.h"

MotorController::MotorController(MyRio_I2c* i2c, uint8_t address)
{
    i2c_ =  i2c;
    address_ = address;
	reset();
    enable();
}
    
MotorController::~MotorController()
{
	reset();
    delay(10);
}
    
void MotorController::enable()
{
    uint8_t writeData = MC_ENABLE;
    I2c_Write(i2c_, address_, &writeData, 1);
    delay(10);
}
    
void MotorController::setInvertState(uint8_t motor, bool state)
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
    
uint16_t MotorController::batteryVoltage()
{
	uint8_t writeData = MC_VOLTAGE;
	I2c_Write(i2c_, address_, &writeData, 1);
	uint8_t readData[2];
	I2c_Read(i2c_, address_, readData, 2);
	delay(10);
	return (readData[0] << 8) + readData[1];
}
    
int32_t MotorController::readEncoderCount(uint8_t motor)
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
		result = (result << 8) + readData[i];
	delay(25);
	return result;
}
    
int32_t MotorController::readEncoderDegree(uint8_t motor)
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
	delay(10);
	return result;
}
    
uint16_t MotorController::readCurrent(uint8_t motor)
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
    
void MotorController::reset()
{
	uint8_t writeData = MC_RESET;
	I2c_Write(i2c_, address_, &writeData, 1);
	delay(10);
}
		
void MotorController::resetEncoder(uint8_t motor)
{
	uint8_t writeData;
	if (motor == 1)
		writeData = MC_E1_RESET;
	else if (motor == 2)
		writeData = MC_E2_RESET;
	else
		return;
	I2c_Write(i2c_, address_, &writeData, 1);
	delay(10);
}
    
void MotorController::resetEncoders()
{
	uint8_t writeData = MC_E12_RESET;
	I2c_Write(i2c_, address_, &writeData, 1);
	delay(10);
}
    
void MotorController::setAddress(uint8_t address)
{
	uint8_t writeData[2] = { MC_SET_ID, address };
	I2c_Write(i2c_, address_, writeData, 2);
	delay(10);
}
    
void MotorController::setMotorPower(uint8_t motor, int8_t power)
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
	delay(10);
}
    
void MotorController::setMotorsPower(int8_t power1, int8_t power2)
{
	uint8_t writeData[3] = { MC_M12_POWER, static_cast<uint8_t>(power1), static_cast<uint8_t>(power2) };
	I2c_Write(i2c_, address_, writeData, 3);
	delay(10);
}
    
void MotorController::setMotorSpeed(uint8_t motor, int16_t speed)
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
	delay(10);
}
    
void MotorController::setMotorsSpeed(int16_t speed1, int16_t speed2)
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
    
void MotorController::setMotorTarget(uint8_t motor, int16_t speed, int32_t target)
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
	delay(10);
}
//  
void MotorController::setMotorsTarget(int16_t speed1, int32_t target1, int16_t speed2, int32_t target2)
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
    
void MotorController::setWDTReset()
{
	uint8_t writeData = MC_WDT_STOP;
	I2c_Write(i2c_, address_, &writeData, 1);
	delay(10);
}
    
void MotorController::setSpeedPID(float p = 1.5, float i = 2.5, float d = 0.008)
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
	
bool MotorController::isBusy(uint8_t motor)
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
    
void MotorController::setTargetPID(float p = 1.5, float i = 0, float d = 0.005)
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
