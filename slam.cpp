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
				if (dist < POINTS_LINE_DIST)
				{
					lines.back().end = i;
					++count;
				}
				else
				{
					--i;
					count = 0;
				}
			//	cout << lines.back().start << "," << lines.back().end << " " << i <<  " -- " << dist << "\n";
			}
			else
			{
				--i;
				count = 0;
			}
		}
	}
	
	for (int i = 0; i < lines.size() - 1; ++i)
	{
		pair<float, float> p1 = lidar->points[lines[i].start], p2 = lidar->points[lines[i].end];
		float dist1 = abs((p2.second - p1.second) * lidar->points[lines[i + 1].start].first - (p2.first - p1.first) * lidar->points[lines[i + 1].start].second + p2.first * p1.second - p2.second * p1.first) / sqrt(pow((p2.second - p1.second), 2) + pow((p2.first - p1.first), 2));
		p1 = lidar->points[lines[i + 1].start], p2 = lidar->points[lines[i + 1].end];
		float dist2 = abs((p2.second - p1.second) * lidar->points[lines[i].end].first - (p2.first - p1.first) * lidar->points[lines[i].end].second + p2.first * p1.second - p2.second * p1.first) / sqrt(pow((p2.second - p1.second), 2) + pow((p2.first - p1.first), 2));
		float dist = (dist1 + dist2) / 2.;
		float cos_ang = (p1.first * p2.first + p1.second * p2.second) / sqrt(pow(p1.first, 2) + pow(p1.second, 2)) / sqrt(pow(p2.first, 2) + pow(p2.second, 2));
		if (dist < 5 && abs(cos_ang - 1) < 0.07)
		{
			lines[i].end = lines[i + 1].end;
			lines.erase(lines.begin() + i + 1, lines.begin() + i + 2);
			--i;
		}
		cout << i << " " << i + 1 << " " << dist << "\n";
	}
		
	for (int i = 0; i < lines.size(); ++i)
	{
		cout << i << ": " << lines[i].start << " " << lines[i].end << "\n";
	}
	
	for (int i = 0; i < lidar->points.size(); ++i)
	{
		circle(map_img, Point(350 + lidar->points[i].first, 350 + lidar->points[i].second), 1, Scalar(0, 0, 0), 3, CV_AA);
	}
	
	for (int i = 0; i < lines.size(); ++i)
	{
		line(map_img,
			Point(350 + lidar->points[lines[i].start].first, 350 + lidar->points[lines[i].start].second),
			Point(350 + lidar->points[lines[i].end].first, 350 + lidar->points[lines[i].end].second),
			Scalar(0, 0, 255),
			2,
			CV_AA);
	}
	
	imwrite("20000015.jpg", map_img);
}
	