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
       // reset();
        enable();
    }
    
    void enable()
    {
        uint8_t writeData = MC_ENABLE;
        I2c_Write(i2c_, address_, &writeData, 1);
        dd(50);
    }

    
    void reset()
    {
        uint8_t writeData = MC_RESET;
        I2c_Write(i2c_, address_, &writeData, 1);
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
    }
    
};