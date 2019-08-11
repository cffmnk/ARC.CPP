#include "localization.h"

std::vector<Position> localization(MyRio_I2c & i2cA, MotorController & mc1, MotorController & mc2)
{
	std::vector<Position> res;
	
	Position pos(0, 0, 0);
	/*
	 *stop next to the wall
	 */
	for (int k = 0; k < 1; ++k)
	{
		Lidar l1;
		
		int bord = 450;
		int cur = 1000;
	
		while (cur == 0 || cur > bord)
		{
			pos = moveRobot(pos, &i2cA, mc1, mc2, 0, -80, 0, false, false);
	
			l1.poll();
			cur = l1.medianInRange(170, 190);
			//std::cout << cur << "\n";
		}
		mc1.setMotorsSpeed(0, 0);
		mc2.setMotorsSpeed(0, 0);
	
		//std::cout << pos.x << " " << pos.y << " " << pos.theta << "\n";
	}
	
	delay(2000);
	
	/*
	 * angle 
	 */
	Lidar l1;
	pos = moveRobot(pos, &i2cA, mc1, mc2, 0, 0, 0, true, true);    // motors reset
	double t = 1;
	while (std::fabs(t) > 0.003)
	{
		l1.poll();
		int i1 = 181;
		int i2 = 179;
		
		while ((l1.ranges[i1 + 1] != 0) && (l1.ranges[i1 + 1] < 700) && i1 < 270)
			++i1;
    
		while ((l1.ranges[i2 - 1] != 0) && (l1.ranges[i2 - 1] < 700) && i2 > 90)
			--i2;
		
		while (((l1.ranges[i1] == 0) || (l1.ranges[i1] >= 700)) && i2 < i1)
			--i1;
		
		while (((l1.ranges[i2] == 0) || (l1.ranges[i2] >= 700)) && i1 > i2)
			++i2;
   
			
		//		std::cout << i2 << " " << i1 << "\n";
		//		std::cout << l1.ranges[i2] << " " << l1.ranges[i1] << "\n";
		//		
		//		for (int i = i2; i <= i1; ++i)
		//			std::cout << l1.ranges[i] << " ";
		//		std::cout << "\n";
		
    
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
        
		//std::cout << "angle " << t * 57.3 << std::endl << std::endl;
		pos = moveRobot(pos, &i2cA, mc1, mc2, 0, 0, 0.5 * t, false, true);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
	
	
	/*
	 *a bit closer to the wall
	 */
	///*
	double dy = 20;
	while (std::abs(dy) > 10)
	{
		l1.poll();
		std::vector<int> asd;
		for (int i = 0; i < 3; ++i)
		{
			asd.push_back(l1.ranges[180 + i]);
			asd.push_back(l1.ranges[180 - i]);
		}
		//std::cout << std::endl;
		std::sort(asd.begin(), asd.end());
        
		dy = std::max(std::min((asd[3] - 270) * -1.0, (double)50), (double) - 50);
		move(&i2cA, mc1, mc2, 0, dy, 0, false);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
	///*
	
	
	//std::cout << "\n" << pos.x << " " << pos.y << " " << pos.theta << "\n";
	
	l1.poll();
	//std::cout << "right " << l1.medianInRange(260, 280) << "\n";
	//std::cout << "left " << l1.medianInRange(80, 100) << "\n";	
	//std::cout << "back " << l1.medianInRange(170, 190) << "\n";
	
	Position mine(l1.medianInRange(80, 100), l1.medianInRange(170, 190), 0);
    
	std::pair<double, double> p2 = rotate(pos.x, pos.y, pos.theta);
	//std::cout << " p2 " << p2.first << " " << p2.second << "\n";
	
	std::pair<double, double> p1 = shift(mine.x, mine.y, p2.first, p2.second);
	
	Position start(p1.first, p1.second, -pos.theta);
	
	std::cout << "current position " << mine.x << " " << mine.y << " " << mine.theta << "\n";
	std::cout << "start positiion " << start.x << " " << start.y << " " << start.theta << "\n";
	
	res.push_back(mine);
	res.push_back(start);
	
	return res;
}