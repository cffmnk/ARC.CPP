#include "taskMain.h"

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
		pii start = current;     // start point
		pii goal(dots[k].x, dots[k].y);     // finish point
		std::vector<pii> points = aStar(start, goal, field);     // build the path
		
		std::cout << "path: " << k << "\n";
		for (int i = 0; i < points.size(); ++i)
			std::cout << (int)points[i].first << " " << (int)points[i].second << '\n';
		std::cout << "\n";
		
		pos = goTo(points, pos, dots[k].theta, &i2c, mc1, mc2);     // get to the point
		
		
		current = goal;
		
		if (k == 4) // last point (finish)
			{
				++k;
				continue;
			}
		
		Lidar lidar;
		shtuka(&i2c, mc1, mc2, &lidar); 
	
		pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, true, true);     // motors reset
		pos = Position(dots[k].x * 115, dots[k].y * 115, dots[k].theta);     // reset position
		
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
		pos = takeCube(pos, &i2c, mc1, mc2, s1, left, change);     // collect object
		
		
		++k;
		
	}
	std::cout << pos.x << " " << pos.y << "\n";
	std::cout << ST.x << " " << ST.y << "\n";
	
	pos = cellShift(&i2c, mc1, mc2, pos, ST, true);
	
	s1.closeLeft();
	s1.closeRight();
	
	pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, true, true);      // motors reset
	Lidar lidar;
	toWall(15, 5, 0, &i2c, mc1, mc2, &lidar);
}