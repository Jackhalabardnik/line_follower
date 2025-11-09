#pragma once
#include <vector>

namespace PIDRatios {
    constexpr double DOWN_SENSOR_BUFFER = 5,
                     UP_SENSOR_BUFFER = 95,
                     MIDDLE_PART = 1,
                     INTER_PART = 25,
                     OUTER_PART = 50,
                     PART_SUM = MIDDLE_PART+INTER_PART+OUTER_PART,
                     MIDDLE_MUL = MIDDLE_PART/PART_SUM,
                     INTER_MUL = INTER_PART/PART_SUM,
                     OUTER_MUL = OUTER_PART/PART_SUM,
                     PROPORTIONAL_MUL = 15,
                     INTEGRAL_MUL = 0.025,
                     DERIVATIVE_MUL = 10;

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
    double proportionalPart = 0, integralPart = 0, derivativePart = 0, lastError = 0;
    bool PIDSkipped = false;
};