#include "controllers.h"

MotorController::MotorController(MyRio_I2c* i2c, uint8_t address)
	: i2c_(i2c)
	, address_(address)
{
	enable();
	setSpeedPID(MC_SPEED_P, MC_SPEED_I, MC_SPEED_D);
}

MotorController::~MotorController()
{
	reset();
}

void MotorController::enable()
{
	uint8_t writeData = MC_ENABLE;
	I2c_Write(i2c_, address_, &writeData, 1);
}

uint16_t MotorController::batteryVoltage()
{
	uint8_t writeData = MC_VOLTAGE;
	I2c_Write(i2c_, address_, &writeData, 1);
	uint8_t readData[2];
	I2c_Read(i2c_, address_, readData, 2);
	return (readData[0] << 8) + readData[1];
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
		result = (result << 8) + readData[3 - i];
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
	return (readData[0] << 8) + readData[1];
}
	
void MotorController::reset()
{
	uint8_t writeData = MC_RESET;
	I2c_Write(i2c_, address_, &writeData, 1);
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
}

void MotorController::resetEncoders()
{
	uint8_t writeData = MC_E12_RESET;
	I2c_Write(i2c_, address_, &writeData, 1);
}
	
void MotorController::setAddress(uint8_t address)
{
	uint8_t writeData[2] = {MC_SET_ID, address};
	I2c_Write(i2c_, address_, writeData, 2);
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
}
	
void MotorController::setMotorsPower(int8_t power1, int8_t power2)
{
	uint8_t writeData[3] = {MC_M12_POWER, static_cast<uint8_t>(power1), static_cast<uint8_t>(power2)};
	I2c_Write(i2c_, address_, writeData, 3);
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
}
	
void MotorController::setMotorsSpeed(int16_t speed1, int16_t speed2)
{
	uint8_t writeData[5];
	writeData[0] = MC_M12_SPEED;
	writeData[1] = speed1 >> 8;
	writeData[2] = speed1 & 0xFF;
	writeData[3] = speed2 >> 8;
	writeData[4] = speed2 & 0xFF;
	I2c_Write(i2c_, address_, writeData, 4);
}

// void MotorController::setMotorTarget(uint8_t motor, int16_t speed, int32_t target)
//	
// void setMotorsTarget(int16_t speed1, int32_t target1, int16_t speed2, int32_t target2);
	
void MotorController::setWDTReset()
{
	uint8_t writeData = MC_WDT_STOP;
	I2c_Write(i2c_, address_, &writeData, 1);
}
	
void MotorController::setSpeedPID(float p, float i, float d)
{
	uint8_t writeData[7];
	uint16_t pk = static_cast<uint16_t >(p * 1000);
	uint16_t ik = static_cast<uint16_t >(i * 1000);
	uint16_t dk = static_cast<uint16_t >(d * 1000);
	writeData[0] = MC_SPEED_PID;
	writeData[1] = pk >> 8;
	writeData[2] = pk & 0xFF;
	writeData[3] = ik >> 8;
	writeData[4] = ik & 0xFF;
	writeData[5] = dk >> 8;
	writeData[6] = dk & 0xFF;
	I2c_Write(i2c_, address_, writeData, 7);
}
	
// void setTargetPID(float p, float i, float d);