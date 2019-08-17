#include "taskMain.h"

void taskMain(MyRio_I2c & i2c, MotorController & mc1, MotorController & mc2, ServoController & s1, cv::VideoCapture & cap, std::vector<std::vector<int16_t>> field)
{
	Position pos(0, 0, 0);
	pos = moveShift(pos, &i2c, mc1, mc2, 0, -450, 250, 20);
	int cnt = 0;
	
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
	
	if (hypotl((dots[2].x - dots[0].x), (dots[2].y - dots[0].y)) < hypotl((dots[0].x - dots[1].x),(dots[0].y - dots[1].y)))
		std::swap(dots[1], dots[2]);
	if (hypotl((dots[3].x - dots[0].x), (dots[3].y - dots[0].y)) < hypotl((dots[0].x - dots[1].x), (dots[0].y - dots[1].y)))
		std::swap(dots[1], dots[3]);
	if (hypotl((dots[3].x - dots[1].x), (dots[3].y - dots[1].y)) < hypotl((dots[2].x - dots[1].x), (dots[2].y - dots[1].y)))
		std::swap(dots[2], dots[3]);
	
	std::vector<pii> points = aStar(current, current, field);       // build the path
	pos = goTo(points, pos, dots[k].theta, &i2c, mc1, mc2);      // get to the point
	
	while (k < 5)
	{
		Lidar lidar;
		grid(&lidar, &field, &pos);
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
		
		
		shtuka(&i2c, mc1, mc2, &lidar); 
	
		pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, true, true);     // motors reset
		pos = Position(dots[k].x * 115, dots[k].y * 115, dots[k].theta);     // reset position
		
		if(cnt == 2 || k == 3)
		{
			cube_color[k] = needed;
			if (k == 1)
			{
				if (cube_color[2] == object_color[3])
					object_color[k] = cube_color[3];
				if (cube_color[3] == object_color[2])
					object_color[k] = cube_color[2];
			}
			else if (k == 2)
			{
				if (cube_color[3] == object_color[1])
					object_color[k] = cube_color[1];
				if (cube_color[1] == object_color[3])
					object_color[k] = cube_color[1];
			}
			else
			{
				object_color[k] = 2;
			}
		}
			
		
		if (cube_color[k] < 1)
		{
			cube_color[k] = checkCube(&cap);
			if (cube_color[k] > 0)
				++cnt;
		}
			
		if (object_color[k] < 1)
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
			else if (k + 2 != 4)
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
			if (cube_color[k + 1] > 0 && cube_color[k + 1] != needed && k == 1)
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
		if (k == 4)
		{
			s1.closeLeft();
			s1.closeRight();
		}
	}
	s1.closeLeft();
	s1.closeRight();
	std::cout << pos.x << " " << pos.y << "\n";
	std::cout << ST.x << " " << ST.y << "\n";
	
	pos = cellShift(&i2c, mc1, mc2, pos, ST, true);

	
	pos = moveRobot(pos, &i2c, mc1, mc2, 0, 0, 0, true, true);      // motors reset
	Lidar lidar;
	toWall(17, 5, 0, &i2c, mc1, mc2, &lidar);
}