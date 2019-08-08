#include "takeCube.h"

Position takeCube(Position &cur, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, ServoController & s1, bool leftArm, bool change)
{
	Position pos = cur;
	double dx1 = 0;
    double dx2 = 0;
    if (leftArm)
    {
        dx1 = 55;
        dx2 = -110;
    }
    else
    {
        dx1 = -55;
        dx2 = 92;
    }

    pos = moveShift(pos, i2c, mc1, mc2, dx1, 20, 100, 20);


    s1.up();
	delay(1000);
	
	if (leftArm)
	{
		s1.closeLeft();
	}
	else
	{
		s1.closeRight();
	}
	delay(400);
	
	if (change)
	{
		pos = moveShift(pos, i2c, mc1, mc2, dx2, 0, 100, 20);
		if (leftArm)
		{
			s1.openRight();
		}
		else
		{
			s1.openLeft();
		}
		delay(400);
	}
	
	s1.down();
	delay(1000);
	
	return pos;
	
}