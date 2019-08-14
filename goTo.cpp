#include "goTo.h"

const int16_t CELL = 115;

Position cellShift(MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, Position cur, Position goal, bool derevo)
{
	Position pos = cur;
	int16_t a = 30; //30;
	int16_t c = 5; //5;
	double b = 0.04;
	if (derevo)
	{
		a = 1;
		c = 1;
		b = 0.01;
	}
	    
	//std::cout << pos.x << " " << pos.y << " " << pos.theta << "\n";
	//std::cout << points[i].first << " "  <<  points[i].second << "\n";
	//std::cout << points[i].first * CELL << " " << points[i].second * CELL << "\n";

	int border = 180;

	int dx = goal.x - pos.x;
	int dy = goal.y - pos.y;
	double dtheta = goal.theta - pos.theta;

	while (std::abs(dx) > a || std::abs(dy) > a || std::abs(dtheta) > b)
	{
		dx = goal.x - pos.x;
		dy = goal.y - pos.y;
		dtheta = goal.theta - pos.theta;

		int x_speed = std::max(std::min(dx * c, border), -border);
		int y_speed = std::max(std::min(dy * c, border), -border);

		if (std::abs(dtheta + 2 * M_PI) < std::abs(goal.theta))
			dtheta += 2 * M_PI;
		else if (std::abs(dtheta - 2 * M_PI) < std::abs(goal.theta))
			dtheta -= 2 * M_PI;

		double theta_speed = std::max(std::min(dtheta, M_PI / 2), -M_PI / 2);

		pos = moveRobot(pos, i2c, mc1, mc2, x_speed, y_speed, theta_speed, false, true);
	        
	}
	
	return pos;
}

Position goTo(std::vector<std::pair<int, int>> & points, Position cur, double theta, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2)
{
    Position pos = cur;
    for (int i = 0; i < (int)points.size(); ++i)
    {
	    bool derevo = (i + 1 == (int)points.size() || i == 0);
	    pos = cellShift(i2c, mc1, mc2, pos, Position(points[i].first * CELL, points[i].second * CELL, theta), derevo);

	    
    }
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
	
	return pos;
}