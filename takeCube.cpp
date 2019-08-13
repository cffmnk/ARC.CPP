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
        dx1 = -60;
        dx2 = 92;
    }

    pos = moveShift(pos, i2c, mc1, mc2, dx1, 20, 90, 5);
	delay(1000);


    s1.up();
	delay(1000);
	
	if (leftArm)
	{
		auto begin = std::chrono::high_resolution_clock::now();
		auto end = std::chrono::high_resolution_clock::now();
		auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
		while (time < 1200)
		{
			pos = moveRobot(pos, i2c, mc1, mc2, -20, 10, 0, false, true);
			end = std::chrono::high_resolution_clock::now();
			time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
			
		}
		
		s1.closeLeft();
		pos = moveShift(pos, i2c, mc1, mc2, 0, 0, 30, 3);
		
	}
	else
	{
		s1.closeRight();
		pos = moveShift(pos, i2c, mc1, mc2, 20, 20, 30, 5);
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
	pos = moveShift(pos, i2c, mc1, mc2, 0, -22, 100, 10);
	delay(1000);
	
	return pos;
}