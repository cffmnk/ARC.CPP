#include "alignment.h"

void toWall(double dis, double precition, int idx, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2)
{
	Lidar l1;
	///*
	double dy = precition + 1;
	while (std::abs(dy) > precition)
	{
		l1.poll();
		std::vector<int> asd;
		for (int i = 0; i < 3; ++i)
		{
			asd.push_back(l1.ranges[i + idx]);
			asd.push_back(l1.ranges[(idx + 359 - i) % 360]);
		}
		// std::cout << std::endl;
		 std::sort(asd.begin(), asd.end());
	    
		std::cout << asd[3] << " " << dy  << "\n";
        
		dy = std::max(std::min((asd[3] - dis) * 10.0, (double)50), (double) - 50);
		move(i2c, mc1, mc2, 0, dy, 0, false);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
}

void alignment(MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2)
{
	Lidar l1;
	///*
	toWall(23, 5, 0, i2c, mc1, mc2);
	//*/
    
	//return;
    ///*
    int mid = 24;
    int dx = mid;
    while (std::abs(dx) >7)
    {
        l1.poll();
        int al = mid;
        for (int i = 5; i < 40; ++i)
        {
            if (l1.ranges[i] > 40 || l1.ranges[i] == 0)
            {
                al = i;
                break;
            }       
        }
        std::cout <<"al " << al << "\n";

        dx = -(mid - al) * 6;
        move(i2c, mc1, mc2, dx, 0, 0, false);
    }
    mc1.setMotorsSpeed(0, 0);
    mc2.setMotorsSpeed(0, 0);
	delay(1000);
    //*/
	
	///*
	double t = 1;
	while (std::fabs(t) > 0.015)
	{
		l1.poll();
		int i1 = 1;
		int i2 = 359;
		
		while ((l1.ranges[i1 + 1] != 0) && (l1.ranges[i1 + 1] < 70) && i1 < 270)
			++i1;
    
		while ((l1.ranges[i2 - 1] != 0) && (l1.ranges[i2 - 1] < 70) && i2 > 90)
			--i2;
		
		while (((l1.ranges[i1] == 0) || (l1.ranges[i1] >= 70)))
		{
			--i1;
			if (i1 < 0)
				i1 += 360;
		}	
		
		while (((l1.ranges[i2] == 0) || (l1.ranges[i2] >= 70)))
		{
			++i2;
			if (i2 >= 360)
				i2 -= 360;
		}
			
   
		std::cout << i2 << " " << i1 << "\n";
		std::cout << l1.ranges[i2] << " " << l1.ranges[i1] << "\n";
				
		//for (int i = i2; i <= i1; ++i)
			//std::cout << l1.ranges[i] << " ";
		//std::cout << "\n";
		
		double a2_x = l1.ranges[i2] * cos(i2 * M_PI / 180);
		double a2_y = l1.ranges[i2] * sin(i2 * M_PI / 180);
		double a1_x = l1.ranges[i1] * cos(i1 * M_PI / 180);
		double a1_y = l1.ranges[i1] * sin(i1 * M_PI / 180);
		double A_X = (a2_x - a1_x);
		double A_Y = (a2_y - a1_y);
        
		if ((A_X) * (A_X) + (A_Y) * (A_Y) == 0 || std::fabs(((A_X) / sqrt((A_X) * (A_X) + (A_Y) * (A_Y))) > 1))
			t = 0;
		else
			t = std::acos((A_X) / sqrt((A_X) * (A_X) + (A_Y) * (A_Y))) - M_PI / 2 + 0.07;
		t = std::max(std::min(t, 0.7), -0.7);
        
		std::cout << "angle " << t * 57.3 << std::endl << std::endl;
		move(i2c, mc1, mc2, 0, 0, 0.5 * t, false);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
	//*/
	
	///*
	toWall(23, 5, 0, i2c, mc1, mc2);
	//*/
    
  //  /*
    mid = 25;
	dx = mid;
	while (std::abs(dx) > 5)
	{
		l1.poll();
		int al = mid;
		for (int i = 5; i < 40; ++i)
		{
			if (l1.ranges[i] > 40 || l1.ranges[i] == 0)
			{
				al = i;
				break;
			}       
		}
		//std::cout << "al " << al << "\n";

		dx = -(mid - al) * 5;
		move(i2c, mc1, mc2, dx, 0, 0, false);
	    
		// Position pos(0, 0, 0);
		// pos = moveShift(pos, i2c, mc1, mc2, dx, 0, 15, 1);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
   // */
	
}