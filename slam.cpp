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

void grid(Lidar* lidar, vector<vector<int>>* f, Position* pos)
{
	lidar->poll();
	for (int i = 1; i < lidar->ranges.size() - 1; ++i)
	{
		if (abs(lidar->ranges[i - 1] - lidar->ranges[i]) > 5 && abs(lidar->ranges[i] - lidar->ranges[i + 1]) > 5)
			lidar->ranges[i] = 0;
	}
	
	for (int i = 0; i < lidar->ranges.size(); ++i)
	{
		auto raw_p = lidar->points[i];
		int x = raw_p.first * cos(pos->theta - M_PI / 2) + raw_p.second * sin(pos->theta - M_PI / 2);
		int y = -raw_p.first * sin(pos->theta - M_PI / 2) + raw_p.second * cos(pos->theta - M_PI / 2);
		if (lidar->ranges[i] > 10)
		{
			x = round(x / 11.5 + pos->x / 115.) + 1;
			y = round(y / 11.5 + pos->y / 115.) + 1;
			cout << x << " " << y << "\n";
			if (x > 0 && x < f->size() && y > 0 && y < f->size())
			{
				
				f->at(y).at(22 - x) = 1;
			}
		}
	}
	
	for (char i = 'A' - 2; i <= 'U'; ++i)
		cout << i << " ";
	cout  << "\n";
	char k = 'A' - 1;
	for (auto i : *f)
	{
		cout << k++ << " ";
		for (auto j : i)
			cout << j << " ";
		cout << "\n";
	}
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
		pair<float, float> p11 = lidar->points[lines[i].start], p12 = lidar->points[lines[i].end];
		float dist1 = abs((p12.second - p11.second) * lidar->points[lines[i + 1].start].first - (p12.first - p11.first) * lidar->points[lines[i + 1].start].second + p12.first * p11.second - p12.second * p11.first) / sqrt(pow((p12.second - p11.second), 2) + pow((p12.first - p11.first), 2));
		pair<float, float> p21 = lidar->points[lines[i + 1].start], p22 = lidar->points[lines[i + 1].end];
		float dist2 = abs((p22.second - p21.second) * lidar->points[lines[i].end].first - (p22.first - p21.first) * lidar->points[lines[i].end].second + p22.first * p21.second - p22.second * p21.first) / sqrt(pow((p22.second - p21.second), 2) + pow((p22.first - p21.first), 2));
		float dist = (dist1 + dist2) / 2.;
		pair<float, float> p = { p12.first - p11.first, p12.second - p11.second };
		pair<float, float> q = { p22.first - p21.first, p22.second - p21.second };
		float cos_ang = (p.first * q.first + p.second * q.second) / sqrt(p.first * p.first + p.second * p.second) / sqrt(q.first * q.first + q.second * q.second);
		if (dist < 5 && abs(cos_ang - 1) < 0.2)
		{
			lines[i].end = lines[i + 1].end;
			lines.erase(lines.begin() + i + 1, lines.begin() + i + 2);
			--i;
		}
		cout << i << " " << i + 1 << " " << dist << " " << cos_ang << "\n";
	}
	
	for (int i = 0; i < lines.size(); ++i)
	{
		if (lines[i].end - lines[i].start < 5)
		{
			lines.erase(lines.begin() + i, lines.begin() + i + 1);
			--i;
		}
	}
	//		
		for(int i = 0 ; i < lines.size() ; ++i)
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
	
	imwrite("scan.jpg", map_img);
	
	vector<vector<float>> grid(21, vector<float>(21, 0));
	
	for (int i = 0; i < lines.size(); ++i)
	{
		auto p1 = lidar->points[lines[i].start];
		auto p2 = lidar->points[lines[i].end];
		
		for (int x = 0; x < 21; ++x)
		{
			for (int y = 0; y < 21; ++y)
			{
				
			}
		}
	}
}
	