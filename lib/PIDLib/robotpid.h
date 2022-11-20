#pragma once
#include <vector>

namespace PIDRatios {
    constexpr double  DOWN_SENSOR_BUFFER = 10, 
                      UP_SENSOR_BUFFER = 90,
                      MID_MUL = 0.4,
                      INTER_MUL = 1,
                      OUTER_MUL = 2;
}

struct RobotEngineSpeed
{
    double leftEngineSpeed, rightEngineSpeed;
};

class RobotPID
{
public:
    RobotPID(double startEngineSpeed, double maxEngineSpeed, double minEngineSpeed);
    RobotEngineSpeed calculatePID(std::vector<double> sensor_values);
private:

    const double maxEngineSpeed, minEngineSpeed;
    double leftEngineSpeed, rightEngineSpeed;
};