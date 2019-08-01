#include "QR.h"

const int8_t N = 23;
const int16_t CELL = 115;

void fill(std::vector<std::vector<int16_t>> & field)
{
	for (int i = 0; i < N; ++i)
	{
		for (int j = 0; j < N; ++j)
		{
			field[i][j] = 0;
		}
	}
	for (int i = 0; i < N; ++i)
	{
		field[1][i] = 6;
		field[i][1] = 6;
		field[N - 2][i] = 6;
		field[i][N - 2] = 6;
	}
	for (int i = 0; i < N; ++i)
	{
		field[0][i] = 1;
		field[i][0] = 1;
		field[N - 1][i] = 1;
		field[i][N - 1] = 1;
	}
}

void QR(Position & pos, std::vector<std::vector<int16_t>> & field)
{
    fill(field);
    
	std::string qr;
    std::vector<int> ddx = {-1, -1, -1, 0, 1, 1, 1, 0};
    std::vector<int> ddy = {-1, 0, 1, 1, 1, 0, -1, -1};
    
    //data = qr.read();
    qr = "(M,O,L,R)(U,L,S,N)(E,K,C,M)(J,D,H,F)";
    
    int16_t start_x1 = (qr[1] - 'A') * CELL;
    int16_t start_y1 = ('U' - qr[3]) * CELL;
    int16_t start_x2 = (qr[5] - 'A') * CELL;
    int16_t start_y2 = ('U' - qr[7]) * CELL;
    
    
    double startX = start_x2 - start_x1;
    double startY = start_y2 - start_y1;
    
    //cout << startX  << " " << startY << endl;
    
    double start_len = std::hypot(startX, startY);
    
    startX /= start_len;
    startY /= start_len;
    
    Pair p1 = Pair(startX * 172.5, startY * 172.5);
    
    Pair p2 = rotate(p1.x, p1.y, M_PI / 2);
    
    double start_cx = p2.x + p1.x + start_x1;
    double start_cy = p2.y + p1.y + start_y1;
    
    double real_theta =  acos((startY) / sqrt((startX) * (startX) + (startY) * (startY))) * (1 - 2 * (startX > 0));
    double real_x = (start_cx + startX * pos.y);
    double real_y = (start_cy + startY * pos.y);
    
    int16_t dx = 1 + round(start_cx  / CELL);
    int16_t dy = 1 + round(start_cy / CELL);
    int16_t ax = dx + round(startX * pos.y / CELL);
    int16_t ay = dy + round(startY * pos.y / CELL);
    
    field[ay][ax] = 12;
    
    for (int i = 1; i < N - 1; ++i)
    {
        for (int j = 1; j < N - 1; ++j)
        {
            if (hypot(dx - i, dy - j) < 3)
            {
                field[j][i] = 2;
            }
        }
    }
    
    double theta[3];
    
    int8_t x1[3];
    int8_t y1[3];
    int8_t x2[3];
    int8_t y2[3];
    
    for (int i = 0; i < 3; ++i)
    {
        x1[i] = std::min((qr[10 + i * 9] - 'A'), (qr[14 + i * 9] - 'A'));
        y1[i] = std::min(('U' - qr[12 + i * 9]), ('U' - qr[16 + i * 9]));
        x2[i] = std::max((qr[10 + i * 9] - 'A'), (qr[14 + i * 9] - 'A'));
        y2[i] = std::max(('U' - qr[12 + i * 9]), ('U' - qr[16 + i * 9]));
    }
    
    for (int k = 0; k < 3; ++k)
    {
        for (int i = x1[k]; i <= x2[k]; ++i)
        {
            for (int j = y1[k]; j <= y2[k]; ++j)
            {
                field[j + 1][i + 1] = 3 + k;
            }
        }
    }
    
    for (int i = 1; i < N - 1; ++i)
    {
        for (int j = 1; j < N - 1; ++j)
        {
            if (field[i][j] < 6 && field[i][j] > 0)
            {
                for (int k = 0; k < 8; ++k)
                {
                    int i1 = i + (int)ddy[k];
                    int j1 = j + (int)ddx[k];
                    if (field[i1][j1] == 0)
                        field[i1][j1] = 6;
                }
            }
        }
    }
    
    for (int k = 0; k < 3; ++k)
    {
        int8_t f_x1 = x1[k];
        int8_t f_y1 = y1[k];
        int8_t f_x2 = 21 - x2[k];
        int8_t f_y2 = 21 - y2[k];
        
        int8_t min_dist = std::min(f_x1, std::min(f_x2, std::min(f_y1, f_y2)));
        
        if (min_dist == f_x1)
        {
            field[1 + f_y1 + 1][1 + 23 - f_x2] = (2 + k) * 10 + 3 + k;
            theta[k] = M_PI / 2;
        }
        else if (min_dist == f_x2)
        {
            field[1 + f_y1 + 1][1 + f_x1 - 2] = (2 + k) * 10 + 3 + k;
            theta[k] = -M_PI / 2;
        }
        else if (min_dist == f_y1)
        {
            field[1 + 23 - f_y2][1 + f_x1 + 1] = (2 + k) * 10 + 3 + k;
            theta[k] = M_PI;
        }
        else
        {
            field[1 + f_y1 - 2][1 + f_x1 + 1] = (2 + k) * 10 + 3 + k;
            theta[k] = 0;
        }
    }
    
    
    pos.x = real_x;
    pos.y = real_y;
    pos.theta = real_theta;
    
    
    int8_t x[4];
    int8_t y[4];
    
    for (int i = 1; i < N - 1; ++i)
    {
        for (int j = 1; j < N - 1; ++j)
        {
            if (field[i][j] / 10 == 0) continue;
            x[field[i][j] / 10 - 1] = j;
            y[field[i][j] / 10 - 1] = i;
        }
    }
}