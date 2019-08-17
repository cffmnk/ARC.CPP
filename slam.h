#pragma once

#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "lidar.h"
#include <algorithm>
#include "moveRobot.h"
#include "QR.h"

#define POINTS_THRES 10
#define POINTS_LINE_DIST 0.5

using namespace std;
using namespace cv;

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
bool cor(int id, int n);
bool isCubeVer(int i, int j, vector<vector<int16_t>>* f);

void buildCubes(vector<vector<int16_t>>* f, Position* pos);

void grid(Lidar* lidar, vector<vector<int16_t>>* f, Position* pos);

void lsf(vector<pair<int, int>>* points, Line* line);

void slam(Lidar* lidar);