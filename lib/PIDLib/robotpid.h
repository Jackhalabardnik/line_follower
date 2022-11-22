#pragma once
#include <vector>

namespace PIDRatios {
    constexpr double  DOWN_SENSOR_BUFFER = 10, 
                      UP_SENSOR_BUFFER = 90,
                      MIDDLE_MUL = 1,
                      INTER_MUL = 2,
                      OUTER_MUL = 3,
                      PROPORTIONAL_MUL = 0.3,
                      INTEGRAL_MUL = 0.01;
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

    const double maxEngineSpeed, minEngineSpeed, idleEngineSpeed;
    double integralPart = 0;
};