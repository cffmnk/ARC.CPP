#include "localization.h"

std::vector<Position> localization(MyRio_I2c & i2c, MotorController & mc1, MotorController & mc2, MyRio_Dio & LED1, cv::VideoCapture & cap)
{
	std::vector<Position> res;
	
	Position pos(0, 0, 0);
	/*
	 *stop next to the wall
	 */
	for (int k = 0; k < 1; ++k)
	{
		Lidar l1;
		
		double bord = 60;
		double cur = 100;
	
		while (cur == 0 || cur > bord)
		{
			pos = moveRobot(pos, &i2c, mc1, mc2, 0, -80, 0, false, false);
	
			l1.poll();
			cur = l1.medianInRange(180, 210);
			std::cout << cur << "\n";
		}
		mc1.setMotorsSpeed(0, 0);
		mc2.setMotorsSpeed(0, 0);
	
		//std::cout << pos.x << " " << pos.y << " " << pos.theta << "\n";
	}
	Dio_WriteBit(&LED1, false);
	
	delay(2000);
	
	/*
	 * angle 
	 */
	Lidar l1;
	pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, true, true);    // motors reset
	double t = 1;
	while (std::fabs(t) > 0.01)
	{
		l1.poll();
		int i1 = 181;
		int i2 = 179;
		
		while ((l1.ranges[i1 + 1] != 0) && (l1.ranges[i1 + 1] < 70) && i1 < 270)
			++i1;
    
		while ((l1.ranges[i2 - 1] != 0) && (l1.ranges[i2 - 1] < 70) && i2 > 90)
			--i2;
		
		while (((l1.ranges[i1] == 0) || (l1.ranges[i1] >= 70)) && i2 < i1)
			--i1;
		
		while (((l1.ranges[i2] == 0) || (l1.ranges[i2] >= 70)) && i1 > i2)
			++i2;
   
			
		std::cout << i2 << " " << i1 << "\n";
		std::cout << l1.ranges[i2] << " " << l1.ranges[i1] << "\n";
				
		for (int i = i2; i <= i1; ++i)
			std::cout << l1.ranges[i] << " ";
		std::cout << "\n";
		
    
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
		pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, -0.5 * t, false, true);
	}
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
	Dio_WriteBit(&LED1, true);
	
	
	/*
	 *a bit closer to the wall
	 */
	///*
	//toWall(27, 10, 180, &i2c, mc1, mc2, lidar);
	
	///*
	Dio_WriteBit(&LED1, false);
	
	Position mine(0, l1.medianInRange(170, 190) * 10, 0);
	std::pair<double, double> p2 = rotate(pos.x, pos.y, pos.theta);
	std::pair<double, double> p1 = shift(mine.x, mine.y, p2.first, p2.second);
	Position start(p1.first, p1.second, -pos.theta);
	
	std::cout << " p2 " << p2.first << " " << p2.second << "\n";
	
	Position cut = mine;
	
	if (l1.medianInRange(80, 100) < l1.medianInRange(260, 280))
	{
		cut.theta += M_PI / 2;
		mine = cellShift(&i2c, mc1, mc2, mine, cut, true);
	//	toWall(27, 10, 0, &i2c, mc1, mc2);
	
		while (!isWall(&cap))
		{
			mine = moveRobot(mine, &i2c, mc1, mc2, -50, 0, 0, false, false);
		}
		mc1.setMotorsSpeed(0, 0);
		mc2.setMotorsSpeed(0, 0);
	
		mine.x += 2300 - l1.medianInRange(350, 10) * 10;
		start.x += mine.x;
	}
	else
	{
		cut.theta -= M_PI / 2;
		mine = cellShift(&i2c, mc1, mc2, mine, cut, true);
		//toWall(27, 10, 0, &i2c, mc1, mc2);
	
		while (!isWall(&cap))
		{
			mine = moveRobot(mine, &i2c, mc1, mc2, 50, 0, 0, false, false);
		}
		mc1.setMotorsSpeed(0, 0);
		mc2.setMotorsSpeed(0, 0);
	
		mine.x += l1.medianInRange(350, 10) * 10;
		start.x += mine.x;
	}
	
	
	std::cout << "right " << l1.medianInRange(260, 280) << "\n";
	std::cout << "left " << l1.medianInRange(80, 100) << "\n" << "\n";
	std::cout << "back " << l1.medianInRange(170, 190) << "\n";
	

	
	std::cout << "current position " << mine.x << " " << mine.y << " " << mine.theta << "\n";
	std::cout << "start positiion " << start.x << " " << start.y << " " << start.theta << "\n";
	std::cout << "\n Position "  << pos.x << " " << pos.y << " " << pos.theta << "\n";
	
	res.push_back(mine);
	res.push_back(start);
	
	return res;
}