#include <cmath>
#include <math.h>

const double ROBOT_RADIUS = 147;
const double WHEEL_RADIUS = 50.8;

const double CONSTANT = 1440;

double x = 0;
double y = 0;
double theta = 0;

class Position
{
 private:
    double x;
    double y;
    double theta;

 public:
    Position(double x_ = 0, double y_ = 0, double theta_ = 0)
    {
        x = x_;
        y = y_;
        theta = theta_;
    }
};

pair<double, double> rotate(double x, double y, double theta)
{
    pair<double, double> res;
    res.first = pos.x * cos(pos.theta) + pos.y * sin(pos.theta);
    res.second = -pos.x * sin(pos.theta) + pos.y * cos(pos.theta);
    return res;
}

double prevEncoders[4] = {0, 0, 0, 0};

Position moveRobot(MyRio_I2c* i2c, MotorController & mc1, MotorController & mc2, Position & cur, bool reset = false, bool relativeToFrame = false)
{
    Position pos = cur;
    if (reset)
    {
        mc1.resetEncoders();
        mc2.resetEncoders();
        for (int i = 0; i < 4; ++i)
            prevEncoders[i] = 0;
        return pos;
    }

    if (relativeToFrame)
    {
        pair<double, double> c1 = rotate(pos.x, pos.y, pos.theta);
        pos.x = c1.first;
        pos.y = c1.second;
    }

    double v[4];
    for (int i = 0; i < 4; ++i)
    {
        double alpha = (2 * i  - 1) * M_PI / 4;
        v[i] = (-sin(alpha) * pos.x + cos(alpha) * pos.y + ROBOT_RADIUS * pos.theta) / WHEEL_RADIUS;
    }

    for (int i = 0; i < 4; ++i)
    {
        v[i] = v[i] / M_PI * 180;
    }

    mc1.setMotorsSpeed(v[3], v[2]);
    mc2.setMotorsSpeed(v[1], v[4]);

    double newEncoders[4] = {0, 0, 0, 0};
    newEncoders[0] = mc1.readEncoderCount(1);
    newEncoders[1] = mc1.readEncoderCount(2);
    newEncoders[2] = mc2.readEncoderCount(1);
    newEncoders[3] = mc2.readEncoderCount(2);

    double dv[4];
    for (int i = 0; i < 4; ++i)
    {
        dv[i] = (2 * M_PI / CONSTANT) * (newEncoders[i] - prevEncoders[i]);
        prevEncoders[i] = newEncoders[i];
    }

    pos.theta = WHEEL_RADIUS * (dv[0] + dv[1] + dv[2] + dv[3]) / (4 * ROBOT_RADIUS);
    pos.x = ((dv[3] - dv[0]) + (dv[2] - dv[1])) / 2 * r / sqrt(2);
    pos.y = ((dv[1] - dv[0]) + (dv[2] - dv[3]) / -2 * r / sqrt(2);

    pair<double, double> c1 = rotate(pos.x, pos.y, -cur.theta);

    pos.x = cur.x + c1.first;
    pos.y = cur.y + c1.second;
    pos.theta += cur.theta;

    return pos;
}