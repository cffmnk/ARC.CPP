#include "path.h"
const double INF = 65000;
const int8_t N = 23;
double f[N][N];

int16_t ddx[8] = {-1, -1, -1, 0, 1, 1, 1, 0};
int16_t ddy[8] = {-1, 0, 1, 1, 1, 0, -1, -1};



V::V(int8_t x_, int8_t y_)
{
    x = x_;
    y = y_;
}

V::V()	
{
	x = 0;
	y = 0;
}

PairI::PairI(int16_t x_, int16_t y_)
{
	x = x_;
	y = y_;
}

bool V::operator < (const V & oth) const
{
    return f[x][y] < f[oth.x][oth.y];
}

bool V::operator ==(const V & oth) const
{
	return f[x][y] == f[oth.x][oth.y];
}

bool V::operator >(const V & oth) const
{
	return f[x][y] > f[oth.x][oth.y];
}


double dis(V a, V b)
{
	return std::hypot(a.x - b.x, a.y - b.y);
}


std::vector<PairI> aStar(V start, V goal, std::vector<std::vector<int16_t>> & field)
{
    std::vector<PairI> res;

    double h[N][N];
    double d[N][N];

    V prev[N][N];
    
    bool used[N][N];
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            d[i][j] = INF;
            h[i][j] = std ::max(std::abs(i - goal.y), std::abs(j - goal.x));
            f[i][j] = d[i][j] + h[i][j];
            used[i][j] = false;
            prev[i][j] = V(-2, -2);
        }
    }

    d[start.y][start.x] = 0;
    f[start.y][start.x] = d[start.y][start.x] + h[start.y][start.x];
    prev[start.y][start.x] = V(-1, -1);

    while (!used[goal.y][goal.x])
    {
	    V cur(-1, -1);
	    
	    for (int i = 0; i < N; ++i)
	    {
		    for (int j = 0; j < N; ++j) 
		    {
			    if (!used[i][j] && (cur == V(-1, -1) || f[i][j] < f[cur.y][cur.x]))
			    {
				    cur = V(j, i);
			    }
		    }
	    }
	   
        used[cur.y][cur.x] = true;
	    
		//std::cout << "Cur " << (int)cur.x << " " << (int)cur.y << " <- " << (int)prev[cur.y][cur.x].x << " " << (int)prev[cur.y][cur.x].y << "\n"
		//std::cout << d[cur.y][cur.x] << " " << h[cur.y][cur.x] << " " << f[cur.y][cur.x] << "\n" << "\n";

        for (int i = 0; i < 8; ++i)
        {
	        V u(cur.x + ddx[i], cur.y + ddy[i]);

            if (u.x < 0 || u.x >= N || u.y < 0 || u.y >= N) continue;
	        if (used[u.y][u.x] || field[u.y][u.x] == 6) continue;

            double newd = d[cur.y][cur.x] + dis(cur, u);
	        
	       // std::cout << "U " << (int)u.x << " " << (int)u.y << " " << newd << " " <<  d[u.y][u.x] << "\n";

            if (newd < d[u.y][u.x])
            {
                prev[u.y][u.x] = cur;
                d[u.y][u.x] = newd;
                f[u.y][u.x] = d[u.y][u.x] + h[u.y][u.x];
            }
        }
    }

    V cur = goal;
    while (cur.x >= 0 || cur.y >= 0)
    {
	    res.push_back(PairI(cur.x, cur.y));
        cur = prev[cur.y][cur.x];
    }

    reverse(res.begin(), res.end());

    return res;
}





