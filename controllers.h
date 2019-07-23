

/*
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

*/
/*
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
*/