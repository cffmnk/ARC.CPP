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

bool cor(int id, int n)
{
	return id >= 0 & id < n;
}

bool isCubeVer(int i, int j, vector<vector<int16_t>>* f)
{
	int n = f->size();
	int m = f->at(0).size();
	if (f->at(i).at(j) != 7) 
		return false;
	if (!cor(i, n) || !cor(i - 1, n) || !cor(i - 2, n)) 
		return false;
	if (f->at(i - 1).at(j) != 7 || f->at(i - 2).at(j) != 7)
		return false;
	if (cor(i + 1, n) && f->at(i + 1).at(j) == 7)
		return false;
	if (cor(i - 3, n) && f->at(i - 3).at(j) == 7)
		return false;
	return true;
}

bool isCubeHor(int i, int j, vector<vector<int16_t>>* f)
{
	int n = f->size();
	int m = f->at(0).size();
	if (f->at(i).at(j) != 7) 
		return false;
	if (!cor(j, m) || !cor(j - 1, m) || !cor(j - 2, m)) 
		return false;
	if (f->at(i).at(j - 1) != 7 || f->at(i).at(j - 2) != 7)
		return false;
	if (cor(j + 1, m) && f->at(i).at(j + 1) == 7)
		return false;
	if (cor(j - 3, m) && f->at(i).at(j - 3) == 7)
		return false;
	return true;
}

void buildCubes(vector<vector<int16_t>>* f, Position* pos)
{
	int n = f->size();
	int m = f->at(0).size();
	int cube = 9;
	std::vector<pii> cubes;
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < m; ++j)
		{
			int i_mid = 0;
			int j_mid = 0;
			if (isCubeHor(i, j, f))
			{
				double y = (n - i - 1) * 115;
				cout << "pos " << i << " " << pos->y << " " << y << "\n";
				if (pos->y > y && cor(i + 2, n))
				{
					i_mid = i + 1;
					j_mid = j - 1;
					for (int i1 = i; i1 <= i + 2; ++i1)
						for (int j1 = j - 2; j1 <= j; ++j1)
							f->at(i1).at(j1) = cube;
				}
				else if (cor(i - 2, n))
				{
					i_mid = i - 1;
					j_mid = j - 1;
				}
			}
			if (isCubeVer(i, j, f))
			{
				double x = j * 115;
				if (pos->x > x && cor(j - 2, m))
				{
					i_mid = i - 1;
					j_mid = j - 1;
				}
				else if (cor(j + 2, m))
				{
					i_mid = i - 1;
					j_mid = j - 1;
				}
			}
			if (i_mid != 0 && j_mid != 0)
			{
				bool ver = cor(i_mid + 2, n) && f->at(i_mid + 2).at(j_mid) == 7 && cor(i_mid - 2, n) && f->at(i_mid - 2).at(j_mid) == 7;
				bool hor = cor(j_mid + 2, m) && f->at(i_mid).at(j_mid + 2) == 7 && cor(j_mid - 2, m) && f->at(i_mid).at(j_mid - 2) == 7;
				if (!ver && !hor)
				{
					for (int i1 = i_mid - 1; i1 <= i_mid + 1; ++i1)
						for (int j1 = j_mid - 1; j1 <= j_mid + 1; ++j1)
							f->at(i1).at(j1) = cube;
				}
				else
				{
					i_mid = 0;
					j_mid = 0;
				}
			}
			if (i_mid != 0 && j_mid != 0)
			{
				cubes.push_back(make_pair(i_mid, j_mid));
			}
		}
	}
	
	std::vector<int> ddx = { -1, -1, -1, 0, 1, 1, 1, 0 };
	std::vector<int> ddy = { -1, 0, 1, 1, 1, 0, -1, -1 };
	
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < m; ++j)
		{
			if (f->at(i).at(j) != cube) continue;
			for (int k = 0; k < 8; ++k)
			{
				int i1 = i + ddy[k];
				int j1 = j + ddx[k];
				if (cor(i1, n) && cor(j1, m) && f->at(i1).at(j1) == 0)
					f->at(i1).at(j1) = 6;		
			}
		}
	}
}

void grid(Lidar* lidar, vector<vector<int16_t>>* f, Position* pos)
{
	lidar->poll();
	for (int i = 1; i < lidar->ranges.size() - 1; ++i)
	{
		if (abs(lidar->ranges[i - 1] - lidar->ranges[i]) > 5 && abs(lidar->ranges[i] - lidar->ranges[i + 1]) > 5)
			lidar->ranges[i] = 0;
		//std::cout << i <<" " << lidar->ranges[i] << "\n";
	}
	
	for (int i = 0; i < 360; ++i)
	{
		if (lidar->ranges[i] < 12) continue;
		int ang = round(pos->theta * 180 / M_PI);
		int idx = (i + ang + 360) % 360;
		
		//cout << i << " " << idx << " " << lidar->ranges[idx] << "\n";	
			
		double x = lidar->ranges[idx] * sin(i * M_PI / 180);
		double y = lidar->ranges[idx] * cos(i * M_PI / 180);
		
		//cout << "coords " << x << " " << y << "\n";  
		
		int xc = round((pos->x + x * 10) / 115) + 1;
		int yc = round((pos->y + y * 10) / 115) + 1;
		
		//cout << "x _ y " << xc << " " << yc << "\n" << "\n";
		
		if (xc > 0 && xc < f->size() && yc > 0 && yc < f->size())
		{
			if (f->at(yc).at(xc) == 0 || f->at(yc).at(xc) == 6)
			{
				f->at(yc).at(xc) = 7;	
			}		
		}
	}
	
	int n = f->size();
	int m = f->at(0).size();
	std::vector<int> ddx = { -1, -1, -1, 0, 1, 1, 1, 0 };
	std::vector<int> ddy = { -1, 0, 1, 1, 1, 0, -1, -1 };
	
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < m; ++j)
		{
			if (f->at(i).at(j) != 7) continue;
			for (int k = 0; k < 8; ++k)
			{
				int i1 = i + ddy[k];
				int j1 = j + ddx[k];
				if (cor(i1, n) && cor(j1, m) && f->at(i1).at(j1) == 0)
					f->at(i1).at(j1) = 6;			
			}
		}
	}
	print_map((*f));
	cout << "\n";
	buildCubes(f, pos);
	buildCubes(f, pos);
}

std::vector<Dot> cubesCoordinates(vector<vector<int16_t>>* f, Position* pos)
{
	int n = f->size();
	int m = f->at(0).size();
	int cube = 9;
	//std::cout << "cubeFound\n";
	std::vector<Dot> res;
	std::vector<int> ddx = { -1, -1, -1, 0, 1, 1, 1, 0 };
	std::vector<int> ddy = { -1, 0, 1, 1, 1, 0, -1, -1 };
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < m; ++j)
		{
			////std::cout << i << " " << j << " " << (f->at(i).at(j)) << "\n" ;
			if (f->at(i).at(j) != cube) continue;
			int cnt = 0;
			
			for (int k = 0; k < 8; ++k)
			{
				int i1 = i + ddy[k];
				int j1 = j + ddx[k];
				if (cor(i1, n) && cor(j1, m) && f->at(i1).at(j1) == cube)
					cnt++;	
			}
			//std::cout << i << " " << j << " " << cnt << "\n";
			if (cnt == 8)
			{
				std::vector<int> dx1 = { 0, 3, 0, -3 };
				std::vector<int> dy1 = { 3, 0, -3, 0 };
				std::vector<double> ang = {M_PI, M_PI / 2, 0, -M_PI / 2};
				int best = 0;
				double dist = 5000;
				//cout << "cube: " << i << " " << j << "\n";
				for (int k = 0; k < 4; ++k)
				{
					int x1 = (j + dx1[k]) * 115;
					int y1 = (n - (i + dy1[k]) - 1) * 115;
					
					int cur = f->at(i + dy1[k]).at(j + dx1[k]);
					
					if (cur == cube || cur == 7 || cur == 6) continue;
					
					if (hypotl(x1 - pos->x, y1 - pos->y) < dist)
					{
						best = k;
						dist = hypotl(x1 - pos->x, y1 - pos->y);
					}
				}
				int xx = j + dx1[best];
				int yy = i + dy1[best];
				f->at(i).at(j) = 20 + res.size();
				//f->at(yy).at(xx) = 10 + res.size();
				/*
				if (f->at(xx + dx1[best] / 3).at(yy + dy1[best] / 3) == 0)
				{
					xx += dx1[best] / 3;
					yy += dy1[best] / 3;
					xx += dx1[best] / 3;
					yy += dy1[best] / 3;
				}
				*/
				std::cout << "cube: " << j << " " << i << "\n";
					
					
				res.push_back(Dot(xx - 1, yy - 1, ang[best]));
			}
		}
	}
	
	return res;
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
	