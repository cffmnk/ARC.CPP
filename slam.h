#pragma once

#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "lidar.h"
#include "moveRobot.h"
#include <algorithm>

#define POINTS_THRES 15
#define POINTS_LINE_DIST 0.7

using namespace std;
using namespace cv;

Position pos(0, 0, 0);

struct Line
{
	int start;
	int end;
	float N;
	float Sxx;
	float Syy;
	float Sxy;
	float Xn;
	float Yn;
	float theta;
	float rho;
};

void lsf(vector<pair<int, int>>* points, Line* line);

void slam(Lidar* lidar);