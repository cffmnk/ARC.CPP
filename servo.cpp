#include "servo.h"
ServoController::ServoController(MyRio_I2c* i2c, uint8_t address)
{
    i2c_ =  i2c;
    address_ = address;
    reset();
    enable();
}
    
ServoController:: ~ServoController()
{
    reset();
    delay(10);
}
    
void ServoController::setAddress(uint8_t address)
{
    uint8_t writeData[2] = {SC_SET_ID, address};
    I2c_Write(i2c_, address_, writeData, 2);
    delay(10);
}
    
uint16_t ServoController::batteryVoltage()
{
    uint8_t writeData = SC_VOLTAGE;
    I2c_Write(i2c_, address_, &writeData, 1);
    uint8_t readData[2];
	delay(10);
    I2c_Read(i2c_, address_, readData, 2);
    delay(10);
    return (readData[0] << 8) + readData[1];
}
    
void ServoController::setWDTReset()
{
    uint8_t writeData = SC_WDT_STOP;
    I2c_Write(i2c_, address_, &writeData, 1);
    delay(10);
}
    
void ServoController::enable()
{
    uint8_t writeData = SC_ENABLE;
    I2c_Write(i2c_, address_, &writeData, 1);
    delay(30);
}
    
void ServoController::reset()
{
    uint8_t writeData = SC_RESET;
    I2c_Write(i2c_, address_, &writeData, 1);
    delay(30);
}
    
void ServoController::setSpeed(uint8_t servo, uint8_t speed)
{
    uint8_t writeData[2];
    if (servo == 1)
        writeData[0] = SC_S1_SPEED;
    else if (servo == 2)
        writeData[0] = SC_S2_SPEED;
    else if (servo == 3)
        writeData[0] = SC_S3_SPEED;
    else if (servo == 4)
        writeData[0] = SC_S4_SPEED;
    else 
        return;
    writeData[1] = speed;
    I2c_Write(i2c_, address_, writeData, 2);
    delay(10);
}
    
void ServoController::setSpeed(uint8_t speed1, uint8_t speed2, uint8_t speed3, uint8_t speed4, uint8_t speed5 = 0, uint8_t speed6 = 0)
{
    uint8_t writeData[7];
    writeData[0] = SC_S16_SPEED;
    writeData[1] = speed1;
    writeData[2] = speed2;
    writeData[3] = speed3;
    writeData[4] = speed4;
    writeData[5] = speed5;
    writeData[6] = speed6;
    I2c_Write(i2c_, address_, writeData, 7);
    delay(10);
}
    
void ServoController::setPosition(uint8_t servo, uint8_t deg)
{
    uint8_t writeData[2];
    if (servo == 1)
        writeData[0] = SC_S1_POSITION;
    else if (servo == 2)
        writeData[0] = SC_S2_POSITION;
    else if (servo == 3)
        writeData[0] = SC_S3_POSITION;
    else if (servo == 4)
        writeData[0] = SC_S4_POSITION;
    else 
        return;
    writeData[1] = deg;
    I2c_Write(i2c_, address_, writeData, 2);
    delay(10);

}

void ServoController::setPositions(uint8_t pos1, uint8_t pos2, uint8_t pos3, uint8_t pos4, uint8_t pos5 = 0, uint8_t pos6 = 0)
{
    uint8_t writeData[7];
    writeData[0] = SC_S16_POSITION;
    writeData[1] = pos1;
    writeData[2] = pos2;
    writeData[3] = pos3;
    writeData[4] = pos4;
    writeData[5] = pos5;
    writeData[6] = pos6;
    I2c_Write(i2c_, address_, writeData, 7);
    delay(10);
}

uint8_t ServoController::readPosition(uint8_t servo)
{
    uint8_t writeData[1];
    if (servo == 1)
        writeData[0] = SC_S1_READ;
    if (servo == 2)
        writeData[0] = SC_S2_READ;
    if (servo == 3)
        writeData[0] = SC_S3_READ;
    if (servo == 4)
        writeData[0] = SC_S4_READ;
    uint8_t res;
    I2c_Write(i2c_, address_, writeData, 1);
	delay(10);
    I2c_Read(i2c_, address_, &res, 1);
    delay(10);
    return res;
}
	
void ServoController::openLeft()
{
	setPosition(1, 70);
}
	
void ServoController::openRight()
{
	setPosition(4, 10);
}
	
void ServoController::closeLeft()
{
	setPosition(1, 7);
}
	
void ServoController::closeRight()
{
	setPosition(4, 75);
}
	
void ServoController::up()
{
	setPosition(2, 140);
	setPosition(3, 0);
}
	
void ServoController::down()
{
	setPosition(2, 25);
	setPosition(3, 95);
}
