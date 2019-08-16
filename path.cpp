#include "path.h"
const double INF = 65000;
const int8_t N = 23;
double f[N][N];

int16_t ddx[8] = {-1, -1, -1, 0, 1, 1, 1, 0};//{0, -1, 0, 1, 0, -1, 0, 1}; //
int16_t ddy[8] = {-1, 0, 1, 1, 1, 0, -1, -1};//{-1, 0, 1, 0, -1, 0, 1, 0}; //

double dis(pii a, pii b)
{
	return std::hypot(a.first - b.first, a.second - b.second);
}

std::vector<pii> aStar(pii start, pii goal, std::vector<std::vector<int16_t>> & field)
{
    std::vector<pii> res;

    double h[N][N];
    double d[N][N];

	pii prev[N][N];
    
    bool used[N][N];
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            d[i][j] = INF;
	        h[i][j] = hypotl((i - goal.second), (j - goal.first));//std::max(std::abs(i - goal.second), std::abs(j - goal.first));
            f[i][j] = d[i][j] + h[i][j];
            used[i][j] = false;
            prev[i][j] = std::make_pair(-2, -2);
        }
    }

    d[start.second][start.first] = 0;
    f[start.second][start.first] = d[start.second][start.first] + h[start.second][start.first];
    prev[start.second][start.first] = std::make_pair(-1, -1);
	
    while (!used[goal.second][goal.first])
    {
	    pii cur(-1, -1);
	    
	    for (int i = 0; i < N; ++i)
	    {
		    for (int j = 0; j < N; ++j) 
		    {
			    if (!used[i][j] && (cur == pii(-1, -1) || f[i][j] < f[cur.second][cur.first]))
			    {
				    cur = pii(j, i);
			    }
		    }
	    }
	    
	    //std::cout << cur.first << " " << cur.second << " " << f[cur.second][cur.first] << "\n";
	   
        used[cur.second][cur.first] = true;
	    
		//std::cout << "Cur " << (int)cur.x << " " << (int)cur.y << " <- " << (int)prev[cur.y][cur.x].x << " " << (int)prev[cur.y][cur.x].y << "\n"
		//std::cout << d[cur.y][cur.x] << " " << h[cur.y][cur.x] << " " << f[cur.y][cur.x] << "\n" << "\n";

        for (int i = 0; i < 8; ++i)
        {
	        pii u(cur.first + ddx[i], cur.second + ddy[i]);

            if (u.first < 0 || u.first >= N || u.second < 0 || u.second >= N) continue;
	        if (used[u.second][u.first]) continue;

            double newd = d[cur.second][cur.first] + dis(cur, u);
	        
	        if (field[u.second + 1][u.first + 1] == 6 || field[u.second + 1][u.first + 1] == 2)
		        newd += 3000;
	        
	       // std::cout << "U " << (int)u.x << " " << (int)u.y << " " << newd << " " <<  d[u.y][u.x] << "\n";

            if (newd < d[u.second][u.first])
            {
                prev[u.second][u.first] = cur;
                d[u.second][u.first] = newd;
                f[u.second][u.first] = d[u.second][u.first] + h[u.second][u.first];
            }
        }
    }
	std::cout << "\n";

    pii cur = goal;
    while (cur.first >= 0 || cur.second >= 0)
    {
	    res.push_back(std::make_pair(cur.first, cur.second));
        cur = prev[cur.second][cur.first];
    }

    reverse(res.begin(), res.end());
	
	std::vector<bool> use((int)res.size(), true);
	for (int i = 1; i + 1 < (int)res.size(); ++i)
	{
		if (res[i].first == res[i + 1].first && res[i].first == res[i - 1].first)
			use[i] = false;
		if (res[i].second == res[i + 1].second && res[i].second == res[i - 1].second)
			use[i] = false;
		if (i + 1 < (int)res.size() && res[i + 1].second - res[i].second == res[i].second - res[i - 1].second && res[i + 1].first - res[i].first == res[i].first - res[i - 1].first)
			use[i] = false;
	}
	std::vector<pii> ans;
	for (int i = 0; i < (int)res.size(); ++i)
	{
		if (use[i])
			ans.push_back(res[i]);
	}

    return ans;
}





