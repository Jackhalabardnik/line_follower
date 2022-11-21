#pragma once
#include <vector>

namespace PIDRatios {
    constexpr double  DOWN_SENSOR_BUFFER = 10, 
                      UP_SENSOR_BUFFER = 90,
                      PROPORTIONAL_MID_MUL = 0.005,
                      PROPORTIONAL_INTER_MUL = 1,
                      PROPORTIONAL_OUTER_MUL = 2,
                      INTEGRAL_MUL = 1;
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
    double leftEngineSpeed, rightEngineSpeed, leftIntegralPart = 0, rightIntegralPart = 0;
};