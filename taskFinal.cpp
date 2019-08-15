#include "taskFinal.h"

mutex mtxCam;

void readCam(VideoCapture* cap, Mat* frame)
{
	mtxCam.lock();
	*cap >> *frame;
	mtxCam.unlock();
}

void taskFinal(MyRio_I2c & i2c, MotorController & mc1, MotorController & mc2, ServoController & s1, cv::VideoCapture & cap)
{
	Mat frame;
	cap.set(CAP_PROP_FRAME_HEIGHT, 240);
	cap.set(CAP_PROP_FRAME_WIDTH, 320);
	thread t(readCam, &cap, &frame);
	
	while (true)
	{
		Mat image, hsv, blue_mask;
		
		mtxCam.lock();
		frame.copyTo(image);
		mtxCam.unlock();
		
		cvtColor(image, hsv, CV_BGR2HSV);
		Color blue(90, 50, 100, 150, 255, 255);
		inRange(hsv, Scalar(blue.lower_h, blue.lower_s, blue.lower_v), Scalar(blue.upper_h, blue.upper_s, blue.upper_v), blue_mask);
		threshold(blue_mask, blue_mask, 40, 255, 0);
		vector<vector<Point>> blue_contours;
		vector<Vec4i> hierarchy;
		int largest_area = 0;
		Rect bounding_rect;
		findContours(blue_mask, blue_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			
		for (size_t i = 0; i < blue_contours.size(); i++) // iterate through each contour.
			{
				double a = contourArea(blue_contours[i], false);          //  Find the area of contour
					
				if(a > largest_area)
				{
					largest_area = a;
					bounding_rect = boundingRect(blue_contours[i]);
				}
			}
			
		rectangle(image, bounding_rect, Scalar(0, 0, 255), 1, 8, 0);
		imwrite("lol.jpg", image);
		
		cout << bounding_rect.x << " " << bounding_rect.y << "\n";
	}
}