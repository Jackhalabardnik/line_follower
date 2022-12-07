#pragma once
#include <vector>

namespace PIDRatios {
    constexpr double DOWN_SENSOR_BUFFER = 10,
                     UP_SENSOR_BUFFER = 90,
                     MIDDLE_MUL = 1,
                     INTER_MUL = 5,
                     OUTER_MUL = 10,
                     PROPORTIONAL_MUL = 0.02,
                     INTEGRAL_MUL = 0.0002,
                     DERIVATIVE_MUL = 0.5;
}

struct RobotEngineSpeed {
    double leftEngineSpeed, rightEngineSpeed;
};

struct PIDParts {
    double proportional, intergal, derivative;
};

class RobotPID {
public:
    RobotPID(double startEngineSpeed, double maxEngineSpeed, double minEngineSpeed);
    RobotEngineSpeed calculatePID(const std::vector<double> &sensor_values);
    void resetPID();
    PIDParts getPIDStatus();
    bool isPIDSkipped();

private:
    bool needToSkipPID(const std::vector<double> &sensor_values);
    double getError(const std::vector<double> &sensor_values);

    const double maxEngineSpeed, minEngineSpeed, idleEngineSpeed;
    double proportionalPart = 0, integralPart = 0, derivativePart = 0;
    bool PIDSkipped = false;
};