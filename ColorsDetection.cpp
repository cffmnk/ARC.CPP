#include "ColorDetection.h"

int checkCube(VideoCapture* cap)
{
	cap->set(CAP_PROP_FRAME_HEIGHT, 240);
	cap->set(CAP_PROP_FRAME_WIDTH, 320);
	
	Mat frame, hsv, orange_mask, yellow_mask, green_mask, blue_mask, red_mask, red_lower_mask, red_upper_mask, result;
	*cap >> frame;
	Mat CubeROI = frame(Rect(250, 120, 50, 50));

	Color orange(10, 100, 100, 20, 255, 255);
	Color red_lower(0, 120, 70, 7, 255, 255);
	Color red_upper(170, 120, 70, 180, 255, 255);
	Color blue(90, 50, 100, 150, 255, 255);
	Color green(31, 20, 20, 85, 255, 255);
	Color yellow(20, 100, 100, 30, 255, 255);
	cvtColor(CubeROI, hsv, CV_BGR2HSV);
	inRange(hsv, Scalar(orange.lower_h, orange.lower_s, orange.lower_v), Scalar(orange.upper_h, orange.upper_s, orange.upper_v), orange_mask);
	inRange(hsv, Scalar(blue.lower_h, blue.lower_s, blue.lower_v), Scalar(blue.upper_h, blue.upper_s, blue.upper_v), blue_mask);
	inRange(hsv, Scalar(green.lower_h, green.lower_s, green.lower_v), Scalar(green.upper_h, green.upper_s, green.upper_v), green_mask);
	inRange(hsv, Scalar(red_lower.lower_h, red_lower.lower_s, red_lower.lower_v), Scalar(red_lower.upper_h, red_lower.upper_s, red_lower.upper_v), red_lower_mask);
	inRange(hsv, Scalar(red_upper.lower_h, red_upper.lower_s, red_upper.lower_v), Scalar(red_upper.upper_h, red_upper.upper_s, red_upper.upper_v), red_upper_mask);
	red_mask = red_lower_mask + red_upper_mask;
	inRange(hsv, Scalar(yellow.lower_h, yellow.lower_s, yellow.lower_v), Scalar(yellow.upper_h, yellow.upper_s, yellow.upper_v), yellow_mask);
	vector<vector<Point>> orange_contours, blue_contours, green_contours, red_contours, yellow_contours;     // Vector for storing contour
    vector<Vec4i> hierarchy;
	int largest_area = 0;
	int largest_contour_index = 0;
	Rect bounding_rect;
	int largest_color = 0;   // 1 - orange, 2 - blue, 3 - green, 4 - red, 5 - yellow, 0 - none
	
	findContours(orange_mask, orange_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(blue_mask, blue_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(green_mask, green_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(red_mask, red_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(yellow_mask, yellow_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	
	for (size_t i = 0; i < orange_contours.size(); i++) // iterate through each contour.
		{
			double a = contourArea(orange_contours[i], false);    //  Find the area of contour
			if(a > largest_area)
			{
				
				largest_area = a;
				largest_color = 1;
			}
		}
	
	
	for (size_t i = 0; i < blue_contours.size(); i++) // iterate through each contour.
		{
			double a = contourArea(blue_contours[i], false);     //  Find the area of contour
			if(a > largest_area)
			{
				largest_area = a;
				largest_color = 2;
			}
		}
	
	for (size_t i = 0; i < green_contours.size(); i++) // iterate through each contour.
		{
			double a = contourArea(green_contours[i], false);      //  Find the area of contour
			if(a > largest_area)
			{
				largest_area = a;
				largest_color = 3;
			}
			
		}
	
	for (size_t i = 0; i < red_contours.size(); i++) // iterate through each contour.
		{
			double a = contourArea(red_contours[i], false);       //  Find the area of contour
			if(a > largest_area)
			{
				largest_area = a;
				largest_color = 4;
			}
		}
	for (size_t i = 0; i < yellow_contours.size(); i++) // iterate through each contour.
		{
			double a = contourArea(yellow_contours[i], false);        //  Find the area of contour
			if(a > largest_area)
			{
				largest_area = a;
				largest_color = 5;
			}
		}
	
	return largest_color;
}


int checkObject(VideoCapture* cap)
{
	cap->set(CAP_PROP_FRAME_HEIGHT, 240);
	cap->set(CAP_PROP_FRAME_WIDTH, 320);
	
	Mat frame, hsv, orange_mask, yellow_mask, green_mask, blue_mask, red_mask, red_lower_mask, red_upper_mask, result;
	*cap >> frame;
	Mat CubeROI = frame(Rect(130, 80, 50, 50));

	Color orange(10, 100, 100, 20, 255, 255);
	Color red_lower(0, 120, 70, 7, 255, 255);
	Color red_upper(170, 120, 70, 180, 255, 255);
	Color blue(90, 20, 100, 150, 255, 255);
	Color green(25, 31, 25, 85, 255, 255);
	Color yellow(20, 70, 70, 30, 255, 255);
	cvtColor(CubeROI, hsv, CV_BGR2HSV);
	inRange(hsv, Scalar(orange.lower_h, orange.lower_s, orange.lower_v), Scalar(orange.upper_h, orange.upper_s, orange.upper_v), orange_mask);
	inRange(hsv, Scalar(blue.lower_h, blue.lower_s, blue.lower_v), Scalar(blue.upper_h, blue.upper_s, blue.upper_v), blue_mask);
	inRange(hsv, Scalar(green.lower_h, green.lower_s, green.lower_v), Scalar(green.upper_h, green.upper_s, green.upper_v), green_mask);
	inRange(hsv, Scalar(red_lower.lower_h, red_lower.lower_s, red_lower.lower_v), Scalar(red_lower.upper_h, red_lower.upper_s, red_lower.upper_v), red_lower_mask);
	inRange(hsv, Scalar(red_upper.lower_h, red_upper.lower_s, red_upper.lower_v), Scalar(red_upper.upper_h, red_upper.upper_s, red_upper.upper_v), red_upper_mask);
	red_mask = red_lower_mask + red_upper_mask;
	inRange(hsv, Scalar(yellow.lower_h, yellow.lower_s, yellow.lower_v), Scalar(yellow.upper_h, yellow.upper_s, yellow.upper_v), yellow_mask);
	vector<vector<Point>> orange_contours, blue_contours, green_contours, red_contours, yellow_contours;     // Vector for storing contour
    vector<Vec4i> hierarchy;
	int largest_area = 0;
	int largest_contour_index = 0;
	Rect bounding_rect;
	int largest_color = 0;   // 1 - orange, 2 - blue, 3 - green, 4 - red, 5 - yellow, 0 - none
	
	findContours(orange_mask, orange_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(blue_mask, blue_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(green_mask, green_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(red_mask, red_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(yellow_mask, yellow_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	
	bitwise_and(CubeROI, CubeROI, result, green_mask);
	
	for (size_t i = 0; i < orange_contours.size(); i++) // iterate through each contour.
		{
			double a = contourArea(orange_contours[i], false);    //  Find the area of contour
			if(a > largest_area)
			{
				
				largest_area = a;
				largest_color = 1;
			}
		}
	
	
	for (size_t i = 0; i < blue_contours.size(); i++) // iterate through each contour.
		{
			double a = contourArea(blue_contours[i], false);     //  Find the area of contour
			if(a > largest_area)
			{
				largest_area = a;
				largest_color = 2;
			}
		}
	
	for (size_t i = 0; i < green_contours.size(); i++) // iterate through each contour.
		{
			double a = contourArea(green_contours[i], false);      //  Find the area of contour
			if(a > largest_area)
			{
				largest_area = a;
				largest_color = 3;
			}
			
		}
	
	for (size_t i = 0; i < red_contours.size(); i++) // iterate through each contour.
		{
			double a = contourArea(red_contours[i], false);       //  Find the area of contour
			if(a > largest_area)
			{
				largest_area = a;
				largest_color = 4;
			}
		}
	for (size_t i = 0; i < yellow_contours.size(); i++) // iterate through each contour.
		{
			double a = contourArea(yellow_contours[i], false);        //  Find the area of contour
			if(a > largest_area)
			{
				largest_area = a;
				largest_color = 5;
			}
		}
	
	imwrite("lol4.jpg", result);
	
	return largest_color;
}

bool isWall(VideoCapture* cap)
{
	cap->set(CAP_PROP_FRAME_HEIGHT, 240);
	cap->set(CAP_PROP_FRAME_WIDTH, 320);
	
	Mat frame, hsv, orange_mask, yellow_mask, green_mask, blue_mask, red_mask, red_lower_mask, red_upper_mask, result;
	*cap >> frame;
	
	
	Mat CubeROI = frame(Rect(215, 130, 60, 30));
	

	Color orange(10, 100, 100, 20, 255, 255);
	Color red_lower(0, 120, 70, 7, 255, 255);
	Color red_upper(170, 120, 70, 180, 255, 255);
	Color blue(90, 20, 100, 150, 255, 255);
	Color green(40, 40, 40, 70, 255, 255);
	Color yellow(20, 100, 100, 30, 255, 255);
	cvtColor(CubeROI, hsv, CV_BGR2HSV);
	inRange(hsv, Scalar(orange.lower_h, orange.lower_s, orange.lower_v), Scalar(orange.upper_h, orange.upper_s, orange.upper_v), orange_mask);
	inRange(hsv, Scalar(blue.lower_h, blue.lower_s, blue.lower_v), Scalar(blue.upper_h, blue.upper_s, blue.upper_v), blue_mask);
	inRange(hsv, Scalar(green.lower_h, green.lower_s, green.lower_v), Scalar(green.upper_h, green.upper_s, green.upper_v), green_mask);
	inRange(hsv, Scalar(red_lower.lower_h, red_lower.lower_s, red_lower.lower_v), Scalar(red_lower.upper_h, red_lower.upper_s, red_lower.upper_v), red_lower_mask);
	inRange(hsv, Scalar(red_upper.lower_h, red_upper.lower_s, red_upper.lower_v), Scalar(red_upper.upper_h, red_upper.upper_s, red_upper.upper_v), red_upper_mask);
	red_mask = red_lower_mask + red_upper_mask;
	inRange(hsv, Scalar(yellow.lower_h, yellow.lower_s, yellow.lower_v), Scalar(yellow.upper_h, yellow.upper_s, yellow.upper_v), yellow_mask);
	vector<vector<Point>> orange_contours, blue_contours, green_contours, red_contours, yellow_contours;      // Vector for storing contour
    vector<Vec4i> hierarchy;
	int largest_area = 0;
	int largest_contour_index = 0;
	Rect bounding_rect;
	int largest_color = 0;    // 1 - orange, 2 - blue, 3 - green, 4 - red, 5 - yellow, 0 - none
	
	findContours(orange_mask, orange_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(blue_mask, blue_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(green_mask, green_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(red_mask, red_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	findContours(yellow_mask, yellow_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	
	for (size_t i = 0; i < orange_contours.size(); i++) // iterate through each contour.
	{
		double a = contourArea(orange_contours[i], false);     //  Find the area of contour
		if(a > largest_area)
		{
				
			largest_area = a;
			largest_color = 1;
		}
	}
	
	
	for (size_t i = 0; i < blue_contours.size(); i++) // iterate through each contour.
	{
		double a = contourArea(blue_contours[i], false);      //  Find the area of contour
		if(a > largest_area)
		{
			largest_area = a;
			largest_color = 2;
		}
	}
	
	for (size_t i = 0; i < green_contours.size(); i++) // iterate through each contour.
	{
		double a = contourArea(green_contours[i], false);       //  Find the area of contour
		if(a > largest_area)
		{
			largest_area = a;
			largest_color = 3;
		}
			
	}
	
	for (size_t i = 0; i < red_contours.size(); i++) // iterate through each contour.
	{
		double a = contourArea(red_contours[i], false);        //  Find the area of contour
		if(a > largest_area)
		{
			largest_area = a;
			largest_color = 4;
		}
	}
	for (size_t i = 0; i < yellow_contours.size(); i++) // iterate through each contour.
	{
		double a = contourArea(yellow_contours[i], false);         //  Find the area of contour
		if(a > largest_area)
		{
			largest_area = a;
			largest_color = 5;
		}
	}
	
	return largest_color == 0;
}

