class Pair
{
private:

public:
    double x;
    double y;
    
    Pair(double x_ = 0, double y_ = 0)
    {
        x = x_;
        y = y_;
    }
};

class Position
{
private:
    
public:
    double x;
    double y;
    double theta;
    Position(double x_ = 0, double y_ = 0, double theta_ = 0)
    {
        x = x_;
        y = y_;
        theta = theta_;
    }
};