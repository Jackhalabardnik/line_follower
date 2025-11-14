#pragma once
#include <vector>

namespace PIDRatios {
    constexpr double DOWN_SENSOR_BUFFER = 5,
                     UP_SENSOR_BUFFER = 95,
                     MIDDLE_PART = 1,
                     INTER_PART = 2,
                     OUTER_PART = 5,
                     PART_SUM = MIDDLE_PART+INTER_PART+OUTER_PART,
                     MIDDLE_MUL = MIDDLE_PART/PART_SUM,
                     INTER_MUL = INTER_PART/PART_SUM,
                     OUTER_MUL = OUTER_PART/PART_SUM,
                     PROPORTIONAL_MUL = 350,
                     INTEGRAL_MUL = 0,
                     DERIVATIVE_MUL = 0,
                     ENGINE_MUL = 0,
                     INTEGRAL_MAX = 1,
                     DERIVATIVE_APLHA = 0.95,
                     D_TIME = 0.0011;

}

struct RobotEngineSpeed {
    double leftEngineSpeed, rightEngineSpeed;
};

struct PIDParts {
    double proportional, intergal, derivative;
};

class RobotPID {
public:
    RobotPID(double maxEngineSpeed, double minEngineSpeed);
    RobotEngineSpeed calculatePID(const std::vector<double> &sensor_values);
    void resetPID();
    PIDParts getPIDStatus();
    bool isPIDSkipped();
    double getLastError();

private:
    bool needToSkipPID(const std::vector<double> &sensor_values);
    double getError(const std::vector<double> &sensor_values);

    const double maxEngineSpeed, minEngineSpeed;
    double proportionalPart = 0, integralPart = 0, derivativePart = 0, lastError = 0;
    bool PIDSkipped = false;
    RobotEngineSpeed lastEngineSpeed = {0,0};
};