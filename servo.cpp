
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

    ServoController(MyRio_I2c* i2c, uint8_t address)
    {
        i2c_ =  i2c;
        address_ = address;
        reset();
        enable();
    }
    
    ~ServoController()
    {
        reset();
        delay(10);
    }
    
    void setAddress(uint8_t address)
    {
        uint8_t writeData[2] = {SC_SET_ID, address};
        I2c_Write(i2c_, address_, writeData, 2);
        delay(10);
    }
    
    uint16_t batteryVoltage()
    {
        uint8_t writeData = SC_VOLTAGE;
        I2c_Write(i2c_, address_, &writeData, 1);
        uint8_t readData[2];
        I2c_Read(i2c_, address_, readData, 2);
        delay(10);
        return (readData[0] << 8) + readData[1];
    }
    
    void setWDTReset()
    {
        uint8_t writeData = SC_WDT_STOP;
        I2c_Write(i2c_, address_, &writeData, 1);
        delay(10);
    }
    
    void enable()
    {
        uint8_t writeData = SC_ENABLE;
        I2c_Write(i2c_, address_, &writeData, 1);
        delay(30);
    }
    
    void reset()
    {
        uint8_t writeData = SC_RESET;
        I2c_Write(i2c_, address_, &writeData, 1);
        delay(30);
    }
    
    void setSpeed(uint8_t servo, uint8_t speed)
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
    
    void setSpeed(uint8_t speed1, uint8_t speed2, uint8_t speed3, uint8_t speed4, uint8_t speed5 = 0, uint8_t speed6 = 0)
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
    
    void setPosition(uint8_t servo, uint8_t deg)
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

    void setPositions(uint8_t pos1, uint8_t pos2, uint8_t pos3, uint8_t pos4, uint8_t pos5 = 0, uint8_t pos6 = 0)
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

    uint8_t readPosition(uint8_t servo)
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
        I2c_Read(i2c_, address_, &res, 1);
        delay(10);
        return res;
    }
	
	void openLeft()
	{
		setPosition(1, 70);
	}
	
	void openRight()
	{
		setPosition(4, 10);
	}
	
	void closeLeft()
	{
		setPosition(1, 7);
	}
	
	void closeRight()
	{
		setPosition(4, 75);
	}
	
	void up()
	{
		setPosition(2, 140);
		setPosition(3, 0);
	}
	
	void down()
	{
		setPosition(2, 25);
		setPosition(3, 95);
	}
};