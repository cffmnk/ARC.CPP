#include "alignment.h"

void toWall(double dis, double precition, int idx, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, Lidar* lidar)
{
	///*
	double dy = precition + 1;
	std::cout << "bbm\n";
	while (std::abs(dy) > precition)
	{
		lidar->poll();
		std::vector<int> asd;
		for (int i = 0; i < 3; ++i)
		{
			asd.push_back(lidar->ranges[i + idx]);
			asd.push_back(lidar->ranges[(idx + 359 - i) % 360]);
		}
		// std::cout << std::endl;
		 std::sort(asd.begin(), asd.end());
	    
		std::cout << asd[3] << " " << dy  << "\n";
        
		dy = std::max(std::min((asd[3] - dis) * 10.0, (double)70), (double) - 70);
		move(i2c, mc1, mc2, 0, dy, 0, false);
	}
	std::cout << "bbm\n";
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
}

void shtuka(MyRio_I2c* i2c, MotorController& mc1, MotorController& mc2, Lidar* lidar)
{
	while (true)
	{
		
		lidar->poll();
		int i1 = 0;
		int i2 = 359;
		
		while ((lidar->ranges[i1 + 1] > 12) && (lidar->ranges[i1 + 1] < 40) && i1 < 270)
			++i1;
    
		while ((lidar->ranges[i2 - 1] > 12) && (lidar->ranges[i2 - 1] < 40) && i2 > 90)
			--i2;
		
		while (((lidar->ranges[i1] <= 12) || (lidar->ranges[i1] >= 40)))
		{
			--i1;
			if (i1 < 0)
				i1 += 360;
		}	
		
		while (((lidar->ranges[i2] <= 12) || (lidar->ranges[i2] >= 40)))
		{
			++i2;
			if (i2 >= 360)
				i2 -= 360;
		}
	
		std::pair<float, float> p1 = lidar->points[i1];
		std::pair<float, float> p2 = lidar->points[i2];
		std::pair<float, float> pm = {
			(p1.first + p2.first) / 2.,
			(p1.second + p2.second) / 2.
		};
		
		double t;
		
		double a2_x = lidar->ranges[i2] * cos(i2 * M_PI / 180);
		double a2_y = lidar->ranges[i2] * sin(i2 * M_PI / 180);
		double a1_x = lidar->ranges[i1] * cos(i1 * M_PI / 180);
		double a1_y = lidar->ranges[i1] * sin(i1 * M_PI / 180);
		double A_X = (a2_x - a1_x);
		double A_Y = (a2_y - a1_y);
        
		if ((A_X) * (A_X) + (A_Y) * (A_Y) == 0 || std::fabs(((A_X) / sqrt((A_X) * (A_X) + (A_Y) * (A_Y))) > 1))
			t = 0;
		else
			t = std::acos((A_X) / sqrt((A_X) * (A_X) + (A_Y) * (A_Y))) - M_PI / 2 + 0.12;
		t = std::max(std::min(t, 0.4), -0.4);
	
		move(i2c, mc1, mc2, (pm.second - 1.5) * 10, (pm.first - 24) * 10, 0.7 * t, 0);
		if (fabs(pm.second - 1.5) < 0.5 && fabs(pm.first - 24) < 0.5 && fabs(t) < 0.01)
			break;
		std::cout << pm.first - 24 << " " << pm.second - 1.5 << "\n";
	}
}

void center(double precition, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, Lidar* lidar)
{
	int dx = precition + 1;
	lidar->clear();
	int dl = -3;
	while (std::abs(dx) > precition)
	{
		lidar->poll();
		int i1 = 0;
		int i2 = 359;
		
		while ((lidar->ranges[i1 + 1] > 12) && (lidar->ranges[i1 + 1] < 40) && i1 < 270)
			++i1;
    
		while ((lidar->ranges[i2 - 1] > 12) && (lidar->ranges[i2 - 1] < 40) && i2 > 90)
			--i2;
		
		while (((lidar->ranges[i1] <= 12) || (lidar->ranges[i1] >= 40)))
		{
			--i1;
			if (i1 < 0)
				i1 += 360;
		}	
		
		while (((lidar->ranges[i2] <= 12) || (lidar->ranges[i2] >= 40)))
		{
			++i2;
			if (i2 >= 360)
				i2 -= 360;
		}
		std::cout << "i_ " << i2 << " " << i1 << "\n";
		i2 = 360 - i2;
		std::cout << "i+ " << i2 << " " << i1 << "\n";

		dx = -(i2 - i1 - dl) * 6;
		dx = std::min(70, std::max(dx, -70));
		move(i2c, mc1, mc2, dx, 0, 0, false);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
}

void alignment(MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, Lidar* lidar)
{
	///*
	toWall(23, 5, 0, i2c, mc1, mc2, lidar);
	//*/
	
	center(5, i2c, mc1, mc2, lidar);
    
	//return;
    ///*
	//*/
	lidar->clear();
	///*
	double t = 1;
	while (std::fabs(t) > 0.01)
	{
		lidar->poll();
		int i1 = 0;
		int i2 = 359;
		
		while ((lidar->ranges[i1 + 1] > 12) && (lidar->ranges[i1 + 1] < 40) && i1 < 270)
			++i1;
    
		while ((lidar->ranges[i2 - 1] > 12) && (lidar->ranges[i2 - 1] < 40) && i2 > 90)
			--i2;
		
		while (((lidar->ranges[i1] <= 12) || (lidar->ranges[i1] >= 40)))
		{
			--i1;
			if (i1 < 0)
				i1 += 360;
		}	
		
		while (((lidar->ranges[i2] <= 12) || (lidar->ranges[i2] >= 40)))
		{
			++i2;
			if (i2 >= 360)
				i2 -= 360;
		}
			
   
		std::cout << i2 << " " << i1 << "\n";
		std::cout << lidar->ranges[i2] << " " << lidar->ranges[i1] << "\n";
				
		//for (int i = i2; i <= i1; ++i)
			//std::cout << l1.ranges[i] << " ";
		//std::cout << "\n";
		
		double a2_x = lidar->ranges[i2] * cos(i2 * M_PI / 180);
		double a2_y = lidar->ranges[i2] * sin(i2 * M_PI / 180);
		double a1_x = lidar->ranges[i1] * cos(i1 * M_PI / 180);
		double a1_y = lidar->ranges[i1] * sin(i1 * M_PI / 180);
		double A_X = (a2_x - a1_x);
		double A_Y = (a2_y - a1_y);
        
		if ((A_X) * (A_X) + (A_Y) * (A_Y) == 0 || std::fabs(((A_X) / sqrt((A_X) * (A_X) + (A_Y) * (A_Y))) > 1))
			t = 0;
		else
			t = std::acos((A_X) / sqrt((A_X) * (A_X) + (A_Y) * (A_Y))) - M_PI / 2 + 0.13;
		t = std::max(std::min(t, 0.7), -0.7);
        
		std::cout << "angle " << t * 57.3 << std::endl << std::endl;
		move(i2c, mc1, mc2, 0, 0,  t, false);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
	//*/
	lidar->clear();
	///*
	toWall(23, 5, 0, i2c, mc1, mc2, lidar);
	//*/
	
	center(5, i2c, mc1, mc2, lidar);
    
    /*
    mid = 27;
	dx = mid;
	while (std::abs(dx) > 5)
	{
		lidar->poll();
		int al = mid;
		for (int i = 5; i < 40; ++i)
		{
			if (lidar->ranges[i] > 40 || lidar->ranges[i] == 0)
			{
				al = i;
				break;
			}       
		}
		//std::cout << "al " << al << "\n";

		 dx = -(mid - al) * 10;
		dx = std::min(70, std::max(dx, -70));
		move(i2c, mc1, mc2, dx, 0, 0, false);
	    
		// Position pos(0, 0, 0);
		// pos = moveShift(pos, i2c, mc1, mc2, dx, 0, 15, 1);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
    */
	
}