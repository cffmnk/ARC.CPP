#pragma once
#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
using namespace cv;
using namespace std;

struct Color
{
	uint8_t lower_h;
	uint8_t lower_s;
	uint8_t lower_v;
	uint8_t upper_h;
	uint8_t upper_s;
	uint8_t upper_v;
	
	Color(uint8_t lh, uint8_t ls, uint8_t lv, uint8_t uh, uint8_t us, uint8_t uv)
		: lower_h(lh)
		, lower_s(ls)
		, lower_v(lv)
		, upper_h(uh)
		, upper_s(us)
		, upper_v(uv) {}
};

int checkCube(VideoCapture* cap);

int checkObject(VideoCapture* cap);

bool isWall(VideoCapture* cap);