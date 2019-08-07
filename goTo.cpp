#include "goTo.h"

const int16_t CELL = 115;

Position goTo(std::vector<std::pair<int, int>> & points, Position cur, double theta, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2)
{
    Position pos = cur;
    for (int i = 0; i < (int)points.size(); ++i)
    {
	    int16_t a = 30;//30;
	    int16_t c = 5;//5;
        if (i + 1 == (int)points.size() || i == 0)
        {
            a = 1;
            c = 1;
        }
	    
	    //std::cout << pos.x << " " << pos.y << " " << pos.theta << "\n";
	    //std::cout << points[i].first << " "  <<  points[i].second << "\n";
	    //std::cout << points[i].first * CELL << " " << points[i].second * CELL << "\n";

        int border = 180;

        int dx = points[i].first * CELL - pos.x;
        int dy = points[i].second * CELL - pos.y;
        double dtheta = theta - pos.theta;

        while (std::abs(dx) > a || std::abs(dy) > a || std::abs(dtheta) > 0.03)
        {
            dx = points[i].first * CELL - pos.x;
            dy = points[i].second * CELL - pos.y;
            dtheta = theta - pos.theta;

            int x_speed = std::max(std::min(dx * c, border), -border);
            int y_speed = std::max(std::min(dy * c, border), -border);

            if (std::abs(dtheta + 2 * M_PI) < std::abs(theta))
                dtheta += 2 * M_PI;
            else if (std::abs(dtheta - 2 * M_PI) < std::abs(theta))
                dtheta -= 2 * M_PI;

            double theta_speed = std::max(std::min(dtheta, M_PI / 2), -M_PI / 2);

            pos = moveRobot(pos, i2c, mc1, mc2, x_speed, y_speed, theta_speed, false, true);
	        
        }
    }
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);
	
	return pos;
}