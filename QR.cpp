#include "QR.h"
using namespace std;
using namespace cv;
using namespace zbar;
const int8_t N = 23;
const int16_t CELL = 115;


// Find and decode barcodes and QR codes
std ::string decode(Mat &im)
{
  
	// Create zbar scanner
	ImageScanner scanner;

	// Configure scanner
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
  
	// Convert image to grayscale
	Mat imGray;
	cvtColor(im, imGray, CV_BGR2GRAY);

	// Wrap image data in a zbar image
	Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);

	// Scan the image for barcodes and QRCodes
	int n = scanner.scan(image);
  
	// Print results
	for(Image::SymbolIterator symbol = image.symbol_begin() ; symbol != image.symbol_end() ; ++symbol)
	{
		decodedObject obj;
    
		obj.type = symbol->get_type_name();
		obj.data = symbol->get_data();
    
		// Print type and data
		cout << "Type : " << obj.type << endl;
		cout << "Data : " << obj.data << endl << endl;
		return obj.data;
		// Obtain location
		for(int i = 0 ; i < symbol->get_location_size() ; i++)
		{
			obj.location.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
		}
   
	}
	return "";
}

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

Dot::Dot()
{
	x = 0;
	y = 0;
	theta = 0;
}

Dot::Dot(int x_, int y_, double theta_)
{
	x = x_;
	y = y_;
	theta = theta_;
}

std::vector<Dot> QR(Position & pos, std::vector<std::vector<int16_t>> & field, cv::VideoCapture & cap)
{
    fill(field);
    
	std::string qr = "";
    std::vector<int> ddx = {-1, -1, -1, 0, 1, 1, 1, 0};
    std::vector<int> ddy = {-1, 0, 1, 1, 1, 0, -1, -1};
    
	//data = qr.read();
	//qr = "(M,O,L,R)(U,L,S,N)(E,K,C,M)(J,D,H,F)";
	
	///*
	cv :: Mat im;
	cap >> im;
	
	while (qr == "")
	{
		qr = decode(im);
	}
	//*/
	
	//qr = "(K,K,M,I)(P,R,N,T)(D,H,F,J)(T,N,R,P)";
	
    
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
    
	std::pair<double, double> p1 = make_pair(startX * 172.5, startY * 172.5);
    
	std::pair<double, double> p2 = rotate(p1.first, p1.second, M_PI / 2);
    
    double start_cx = p2.first + p1.first + start_x1;
    double start_cy = p2.second + p1.second + start_y1;
    
    double real_theta =  acos((startY) / sqrt((startX) * (startX) + (startY) * (startY))) * (1 - 2 * (startX > 0));
    double real_x = (start_cx + startX * pos.y);
    double real_y = (start_cy + startY * pos.y);
    
    int16_t dx = 1 + round(start_cx / CELL);
    int16_t dy = 1 + round(start_cy / CELL);
    int16_t ax = dx + round(startX * pos.y / CELL);
    int16_t ay = dy + round(startY * pos.y / CELL);
    
	cout << ay << " " << ax << "\n";
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
    
    double theta[4];
    
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
        for (int i = x1[k]; i <= x2[k] && i < N; ++i)
        {
            for (int j = y1[k]; j <= y2[k] && j < N; ++j)
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
	
	int8_t x[4];
	int8_t y[4];
    
    for (int k = 0; k < 3; ++k)
    {
        int8_t f_x1 = x1[k];
        int8_t f_y1 = y1[k];
        int8_t f_x2 = 21 - x2[k];
        int8_t f_y2 = 21 - y2[k];
        
        int8_t min_dist = std::min(f_x1, std::min(f_x2, std::min(f_y1, f_y2)));
	    
	    int i_idx = 0;
	    int j_idx = 0;
        
        if (min_dist == f_x1)
        {
	        i_idx = 1 + f_y1 + 1;
	        j_idx = 1 + 23 - f_x2;
            theta[k + 1] = M_PI / 2;
        }
        else if (min_dist == f_x2)
        {
	        i_idx = 1 + f_y1 + 1;
	        j_idx = 1 + f_x1 - 2;
            theta[k + 1] = -M_PI / 2;
        }
        else if (min_dist == f_y1)
        {
	        i_idx = 1 + 23 - f_y2;
	        j_idx = 1 + f_x1 + 1;
            theta[k + 1] = M_PI;
        }
        else
        {
	        i_idx = 1 + f_y1 - 2;
	        j_idx = 1 + f_x1 + 1;
            theta[k + 1] = 0;
        }
	    field[i_idx][j_idx] = (2 + k) * 10 + 3 + k;
	    x[k + 1] = j_idx - 1;
	    y[k + 1] = i_idx - 1;
    }
    
    pos.x = real_x;
    pos.y = real_y;
    pos.theta = real_theta;
	theta[0] = real_theta;
	x[0] = ax - 1;
	y[0] = ay - 1;
	
	std::vector<Dot> ans(4);
	for (int i = 0; i < 4; ++i)
	{
		ans[i] = Dot(x[i], y[i], theta[i]);
	}
	field[ay][ax] = 12;
	return ans;
	
}

void print_map(std::vector<std::vector<int16_t>> & field)
{
	std::cout << (char)(0 + 'A' - 1) << " ";
	for (int j = 0; j < N; ++j)
	{
		std::cout.width(3);
		std::cout << (char)(j + 'A' - 1);
	}
	std::cout << "\n";
	for (int i = 0; i < N; ++i)
	{
		std::cout << (char)(i + 'A' - 1) << " ";
		for (int j = 0; j < N; ++j)
		{
			std::cout.width(3);
			std::cout << field[N - 1 - i][j];
		}
		std::cout << "\n";
	}
	return;
}