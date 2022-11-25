#pragma once
#include <vector>
#include "pidstatus.h"

namespace PIDRatios {
    constexpr double  DOWN_SENSOR_BUFFER = 10, 
                      UP_SENSOR_BUFFER = 90,
                      MIDDLE_MUL = 1,
                      INTER_MUL = 2,
                      OUTER_MUL = 3,
                      PROPORTIONAL_MUL = 0.3,
                      INTEGRAL_MUL = 0.01,
                      LIGHT_CURVE_VAL = 20,
                      HEAVY_CURVE_VAL = 70;
}

struct RobotEngineSpeed
{
    double leftEngineSpeed, rightEngineSpeed;
};

class RobotPID
{
public:
    RobotPID(double startEngineSpeed, double maxEngineSpeed, double minEngineSpeed);
    RobotEngineSpeed calculatePID(const std::vector<double> &sensor_values);
    PIDStatus getPIDStatus();
private:
    bool needToSkipPID(const std::vector<double> &sensor_values);
    double getError(const std::vector<double> &sensor_values);
    void updatePIDStatus(const std::vector<double> &sensor_values);

    PIDStatus pidStatus = PIDStatus::PROPORTIONAL;
    const double maxEngineSpeed, minEngineSpeed, idleEngineSpeed;
    double integralPart = 0;
};