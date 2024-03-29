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

