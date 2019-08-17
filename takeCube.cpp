#include "takeCube.h"

Position takeCube(Position &cur, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, ServoController & s1, bool leftArm, bool change)
{
	Position pos = cur;
	pos = moveRobot(pos, i2c, mc1, mc2, 0, 0, 0, true, true); 
	double dx1 = 0;
    double dx2 = 0;
	double dy = 0;
    if (leftArm)
    {
        dx1 = 55;
        dx2 = -80;
	    dy = 22;
    }
    else
    {
        dx1 = -60;
        dx2 = 78;
	    dy = 25;
    }

    pos = moveShift(pos, i2c, mc1, mc2, dx1, dy, 90, 4);

    s1.up();
	delay(1000);
	
	if (leftArm)
	{
		auto begin = std::chrono::high_resolution_clock::now();
		auto end = std::chrono::high_resolution_clock::now();
		auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
		while (time < 1200)
		{
			pos = moveRobot(pos, i2c, mc1, mc2, -20, 6, 0, false, false);
			end = std::chrono::high_resolution_clock::now();
			time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
		}
		s1.closeLeft();
		pos = moveShift(pos, i2c, mc1, mc2, 0, 0, 30, 3);
	}
	else
	{
		auto begin = std::chrono::high_resolution_clock::now();
		auto end = std::chrono::high_resolution_clock::now();
		auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
		while (time < 1200)
		{
			pos = moveRobot(pos, i2c, mc1, mc2, 20, 5, 0, false, false);
			end = std::chrono::high_resolution_clock::now();
			time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
		}
		s1.closeRight();
		pos = moveShift(pos, i2c, mc1, mc2, 0, 0, 30, 3);
	}
	delay(400);
	
	if (change)
	{
		pos = moveShift(pos, i2c, mc1, mc2, dx2, 1, 90, 10);
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
	pos = moveShift(pos, i2c, mc1, mc2, 0, -dy, 100, 10);
	delay(1000);
	
	return pos;
}