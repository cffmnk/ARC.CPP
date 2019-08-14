#include "moveRobot.h"
#include "path.h"
#include "QR.h"
#include "goTo.h"
#include "alignment.h"
#include "takeCube.h"
#include "ColorDetection.h"

double prevEncoders[4] = { 0, 0, 0, 0 };

Position::Position(double x_, double y_ = 0, double theta_ = 0)
{
	x = x_;
	y = y_;
	theta = theta_;
}


std::pair<double, double> rotate(double x, double y, double theta)
{
	std::pair<double, double> res(0, 0);
	res.first = x * cos(theta) + y * sin(theta);
	res.second = -x * sin(theta) + y * cos(theta);
	return res;
}

std::pair<double, double> shift(double x, double y, double dx, double dy)
{
	return std::make_pair(x - dx, y - dy);
}

Position moveRobot(Position cur, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, double x, double y, double theta, bool reset = false, bool relativeToFrame = false)
{
	Position pos = Position(x, y, theta);
	if (reset)
	{
		mc1.resetEncoders();
		mc2.resetEncoders();
		for (int i = 0; i < 4; ++i)
			prevEncoders[i] = 0;
		return cur;
	}

	if (relativeToFrame)
	{
		std::pair<double, double> c1 = rotate(pos.x, pos.y, cur.theta);
		pos.x = c1.first;
		pos.y = c1.second;
	}

	double v[4];
	for (int i = 0; i < 4; ++i)
	{
		double alpha = (2 * i + 1) * M_PI / 4;
		v[i] = (-sin(alpha) * pos.x + cos(alpha) * pos.y + ROBOT_RADIUS * pos.theta) / WHEEL_RADIUS;
	}

	for (int i = 0; i < 4; ++i)
	{
		v[i] = v[i] / M_PI * 180;
	}
	//std::cout << "V: " << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << "\n";

    mc1.setMotorsSpeed(v[2], v[1]);
	mc2.setMotorsSpeed(v[0], v[3]);

	double newEncoders[4] = { 0, 0, 0, 0 };
	
	newEncoders[0] = mc2.readEncoderCount(1);
	newEncoders[1] = mc1.readEncoderCount(2);
	newEncoders[2] = mc1.readEncoderCount(1);
	newEncoders[3] = mc2.readEncoderCount(2);

	double dv[4];
	//std::cout << "Enc ";
    for(int i = 0 ; i < 4 ; ++i)
	{
		dv[i] = (2 * M_PI / CONSTANT) * (newEncoders[i] - prevEncoders[i]);
		//std::cout << newEncoders[i] << " ";
		prevEncoders[i] = newEncoders[i];
	    
	}
	//std::cout << "\n";

    pos.theta = WHEEL_RADIUS * (dv[0] + dv[1] + dv[2] + dv[3]) / (4 * ROBOT_RADIUS);
	pos.x = ((dv[3] - dv[0]) + (dv[2] - dv[1])) / 2 * WHEEL_RADIUS / sqrt(2);
	pos.y = ((dv[1] - dv[0]) + (dv[2] - dv[3])) / -2 * WHEEL_RADIUS / sqrt(2);

	std::pair<double, double> c1 = rotate(pos.x, pos.y, -cur.theta);

	pos.x = cur.x + c1.first;
	pos.y = cur.y + c1.second;
	pos.theta += cur.theta;

	return pos;
}

Position moveShift(Position &cur, MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, double x_shift, double y_shift, double max_speed, double precision) 
{
	Position pos = cur;

	std::pair<double, double> c1 = rotate(x_shift, y_shift, -pos.theta);
	std::pair<double, double> c2 = std::make_pair(cur.x + c1.first, cur.y + c1.second);
	pos = cur;
	
	//std::cout << " C2 " << c2.first << " " << c2.second << "\n";

	double x_speed = std::max(std::min((c2.first - pos.x) * 3, max_speed), -max_speed); 
	double y_speed = std::max(std::min((c2.second - pos.y) * 3, max_speed), -max_speed);
	
	while ((std::abs(x_speed) > precision) || (std::abs(y_speed) > precision))
	{
		x_speed = std::max(std::min((c2.first - pos.x) * 3, max_speed), -max_speed);
		y_speed = std::max(std::min((c2.second - pos.y) * 3, max_speed), -max_speed);
		pos = moveRobot(pos, i2c, mc1, mc2, x_speed, y_speed, 0, false, true);
		//std::cout << "sp " << x_speed << " " << y_speed << "\n";
		//std::cout << "pos " << pos.x << " " << pos.y << "\n";
	}
	
	mc1.setMotorsSpeed(0, 0);
	mc2.setMotorsSpeed(0, 0);

	return pos;
}

void move(MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, double x, double y, double theta, bool reset)
{
	Position pos = Position(x, y, theta);
	if (reset)
	{
		mc1.resetEncoders();
		mc2.resetEncoders();
		for (int i = 0; i < 4; ++i)
			prevEncoders[i] = 0;
		return;
	}

	double v[4];
	//std::cout << "theta " << theta << "\n";
	for(int i = 0 ; i < 4 ; ++i)
	{
		double alpha = (2 * i + 1) * M_PI / 4;
		v[i] = (-sin(alpha) * pos.x + cos(alpha) * pos.y + ROBOT_RADIUS * pos.theta) / WHEEL_RADIUS;
	}

	for (int i = 0; i < 4; ++i)
	{
		v[i] = v[i] / M_PI * 180;
	}
	//std::cout << "V: " << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << "\n";

    mc1.setMotorsSpeed(v[2], v[1]);
	mc2.setMotorsSpeed(v[0], v[3]);

	return;
}

void taskMain(MyRio_I2c & i2c, MotorController & mc1, MotorController & mc2, ServoController & s1, cv::VideoCapture & cap, std::vector<std::vector<int16_t>> field)
{
	
	Position pos(0, 0, 0);
	pos = moveShift(pos, &i2c, mc1, mc2, 0, -400, 250, 20);
	
	s1.openLeft();
	s1.openRight();
	
	std::vector<Dot> dots = QR(pos, field, cap);

	Position ST = pos;
	
	dots.push_back(dots[0]);
	
	cout << "\n";
	for (int i = 0; i < 5; ++i)
	{
		std::cout << dots[i].x <<  " " << dots[i].y << " " << dots[i].theta << std::endl;
	}
	cout << "\n";
	
	print_map(field);
	
	pii current = pii(dots[0].x, dots[0].y);
	
	int cube_color[6];
	int object_color[6];
	
	for (int i = 0; i < 6; ++i)
	{
		cube_color[i] = -1;
		object_color[i] = -1;
	}
	int k = 1;
	
	int needed = 2;
	
	if (abs(dots[2].x - dots[0].x) + abs(dots[2].y - dots[0].x) < abs(dots[0].x - dots[1].x) + abs(dots[0].y - dots[1].x))
		std::swap(dots[1], dots[2]);
	if (abs(dots[3].x - dots[0].x) + abs(dots[3].y - dots[0].x) < abs(dots[0].x - dots[1].x) + abs(dots[0].y - dots[1].x))
		std::swap(dots[1], dots[3]);
	if (abs(dots[3].x - dots[1].x) + abs(dots[3].y - dots[1].x) < abs(dots[2].x - dots[1].x) + abs(dots[2].y - dots[1].x))
		std::swap(dots[2], dots[3]);
	
	while (k < 5)
	{
		if (k == 4)
			dots[4].theta = dots[3].theta;
		pii start = current;    // start point
		pii goal(dots[k].x, dots[k].y);    // finish point
		std::vector<pii> points = aStar(start, goal, field);    // build the path
		
		std::cout << "path: " << k << "\n";
		for (int i = 0; i < points.size(); ++i)
			std::cout << (int)points[i].first << " " << (int)points[i].second << '\n';
		std::cout << "\n";
		
		pos = goTo(points, pos, dots[k].theta, &i2c, mc1, mc2);    // get to the point
		
		
		current = goal;
		
		if (k == 4) // last point (finish)
			{
				++k;
				continue;
			}
		
		Lidar lidar;
		shtuka(&i2c, mc1, mc2, &lidar); 
	
		pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, true, true);    // motors reset
		pos = Position(dots[k].x * 115, dots[k].y * 115, dots[k].theta);    // reset position
		
		cube_color[k] = checkCube(&cap);
		object_color[k] = checkObject(&cap);
		std::cout << "\n";
		std::cout << "colors : " << k << " | " << cube_color[k] << " " << object_color[k] << "\n";
		std::cout << "\n";
		
		if (cube_color[k] != needed)
		{
			if (object_color[k + 1] < 0)
			{
				std::swap(dots[k], dots[k + 1]);
				std::swap(object_color[k], object_color[k + 1]);
				std::swap(cube_color[k], cube_color[k + 1]);
			}
			else
			{
				std::swap(dots[k], dots[k + 2]);
				std::swap(object_color[k], object_color[k + 2]);
				std::swap(cube_color[k], cube_color[k + 2]);
			}
			continue;
		}
		else
		{
			needed = object_color[k];
			if (cube_color[k + 1] > 0 && cube_color[k + 1] != needed)
			{
				std::swap(dots[k + 1], dots[k + 2]);
				std::swap(object_color[k + 1], object_color[k + 2]);
				std::swap(cube_color[k + 1], cube_color[k + 2]);
			}
		}
		
		 
		bool left = ((k == 1) || (k == 3));
		bool change = (k > 1);
		
		pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, true, true); 
		pos = takeCube(pos, &i2c, mc1, mc2, s1, left, change);    // collect object
		
		
		++k;
		
	}
	std::cout << pos.x << " " << pos.y << "\n";
	std::cout << ST.x << " " << ST.y << "\n";
	
	pos = cellShift(&i2c, mc1, mc2, pos, ST, true);
	
	s1.closeLeft();
	s1.closeRight();
	
	pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, true, true);     // motors reset
	Lidar lidar;
	toWall(15, 5, 0, &i2c, mc1, mc2, &lidar);
}