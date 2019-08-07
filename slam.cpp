#include "slam.h"

void lsf(vector<pair<float, float>>* points, Line* line)
{
	line->Sxx = 0;
	line->Syy = 0;
	line->Sxy = 0;
	line->N = points->size();
	line->Xn = 0;
	for (auto i : *points)
	{
		line->Sxx += i.first * i.first;
		line->Syy += i.second * i.second;
		line->Sxy += i.first * i.second;
		line->Xn += i.first;
		line->Yn += i.second;
	}
	line->Xn /= line->N;
	line->Yn /= line->N;
	
	line->theta = 0.5 * atan2(-4 * (line->Sxy - line->N * line->Xn * line->Yn), (line->Syy - line->Sxx) - (line->Yn * line->Yn - line->Xn * line->Xn));
	line->rho = line->Xn * cos(line->theta) + line->Yn * sin(line->theta);
}

void slam(Lidar* lidar)
{
	Mat map_img = Mat::zeros(700, 700, CV_8UC3);
	map_img = Scalar(255, 255, 255);
	
	
	vector<pair<float, float>> p = vector<pair<float, float>>(lidar->points.begin() + 250, lidar->points.begin() + 288);
	vector<Line> lines;
	int count = 0;
	for (int i = 0; i < lidar->ranges.size() - 2; ++i)
	{
		if (count == 0)
		{
			if (lidar->ranges[i] > 10 && abs(lidar->ranges[i] - lidar->ranges[i + 1]) < POINTS_THRES && abs(lidar->ranges[i + 1] - lidar->ranges[i + 2]) < POINTS_THRES)
			{
				Line tmp;
				tmp.start = i;
				tmp.end = i + 2;
				lines.push_back(tmp);
				count = 3;
			}
		}
		else
		{
			if (abs(lidar->ranges[i] - lidar->ranges[lines.back().end]) < POINTS_THRES)
			{
				pair<float, float> p1 = lidar->points[lines.back().start], p2 = lidar->points[lines.back().end];
				float dist = abs((p2.second - p1.second) * lidar->points[i].first - (p2.first - p1.first) * lidar->points[i].second + p2.first * p1.second - p2.second * p1.first) / sqrt(pow((p2.second - p1.second), 2) + pow((p2.first - p1.first), 2));
				if (dist < POINTS_LINE_DIST || lines.back().end - lines.back().start < 5)
				{
					lines.back().end = i;
					++count;
				}
				else
				{
					--i;
					count = 0;
				}
				cout << lines.back().start << "," << lines.back().end << " " << i <<  " -- " << dist << "\n";
			}
			else
			{
				--i;
				count = 0;
			}
		}
	}
	
	vector<pair<pair<int, int>, pair<int, int>>> lines_f;
	for (auto i : lines)
	{
		lines_f.emplace_back();
		lines_f.back().first.first = (lidar->points[i.start].first * cos(pos.theta) - lidar->points[i.start].second * sin(pos.theta));
		lines_f.back().first.second = (lidar->points[i.start].second * cos(pos.theta) + lidar->points[i.start].first * sin(pos.theta));
		lines_f.back().second.first = (lidar->points[i.end].first * cos(pos.theta) - lidar->points[i.end].second * sin(pos.theta));
		lines_f.back().second.second = (lidar->points[i.end].second * cos(pos.theta) + lidar->points[i.end].first * sin(pos.theta));
	}
	
	for (int i = 0; i < lines.size(); ++i)
	{
		cout << i << ": " << lines[i].start << " " << lines[i].end << "\n";
	}
	
	for (int i = 0; i < lidar->points.size(); ++i)
	{
		circle(map_img, Point(lidar->points[i].first, lidar->points[i].second), 1, Scalar(0, 0, 0), 3, CV_AA);
	}
	
	for (int i = 0; i < lines.size(); ++i)
	{
		line(map_img,
			Point(lidar->points[lines[i].start].first, lidar->points[lines[i].start].second),
			Point(lidar->points[lines[i].end].first, lidar->points[lines[i].end].second),
			Scalar(0, 0, 255),
			3,
			CV_AA);
	}
	
	imwrite("0000012.jpg", map_img);
}
	