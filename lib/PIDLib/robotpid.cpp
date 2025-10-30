#include "robotpid.h"
#include "utils.h"

#include <math.h>

RobotPID::RobotPID(double startEngineSpeed, double maxEngineSpeed, double minEngineSpeed)
    : maxEngineSpeed(maxEngineSpeed),
      minEngineSpeed(minEngineSpeed),
      idleEngineSpeed(startEngineSpeed) {}

PIDParts RobotPID::getPIDStatus() {
    return {proportionalPart, integralPart, derivativePart};
}

bool RobotPID::isPIDSkipped() {
    return PIDSkipped;
}

void RobotPID::resetPID() {
    integralPart = 0;
    derivativePart = 0;
    lastError = 0;
}

RobotEngineSpeed RobotPID::calculatePID(const std::vector<double> &sensor_values) {
    PIDSkipped = needToSkipPID(sensor_values);
    PIDSkipped = false;

    if (PIDSkipped) {
        proportionalPart = 0;
    } else {
        double error = getError(sensor_values);
        proportionalPart = PIDRatios::PROPORTIONAL_MUL * error;
        integralPart += error * PIDRatios::INTEGRAL_MUL;
        derivativePart = error; //(error - lastError) * PIDRatios::DERIVATIVE_MUL;
        lastError = error;
    }

    double sum = proportionalPart; // + integralPart + derivativePart;

    double leftEngineSpeed = maxEngineSpeed;
    double rightEngineSpeed = maxEngineSpeed;

    if(sum < 0) {
        rightEngineSpeed += maxEngineSpeed * (sum/maxEngineSpeed);
    }
    if(sum > 0) {
        leftEngineSpeed -= maxEngineSpeed * (sum/maxEngineSpeed);
    }

    bound_value(leftEngineSpeed, minEngineSpeed, maxEngineSpeed);
    bound_value(rightEngineSpeed, minEngineSpeed, maxEngineSpeed);

    return {leftEngineSpeed, rightEngineSpeed};
}

bool RobotPID::needToSkipPID(const std::vector<double> &sensor_values) {
    bool is_all_white = true, is_all_dark = true;

    for (const auto &sensor_value: sensor_values) {
        if (sensor_value > PIDRatios::DOWN_SENSOR_BUFFER) {
            is_all_white = false;
        }
        if (sensor_value < PIDRatios::UP_SENSOR_BUFFER) {
            is_all_dark = false;
        }
    }

    bool are_middle_sensors_dark = sensor_values[2] > PIDRatios::UP_SENSOR_BUFFER && sensor_values[3] > PIDRatios::UP_SENSOR_BUFFER;
    bool are_middle_sensors_white = sensor_values[2] < PIDRatios::DOWN_SENSOR_BUFFER && sensor_values[3] < PIDRatios::DOWN_SENSOR_BUFFER;
    bool are_middle_sensors_close = std::abs(sensor_values[2] - sensor_values[3]) < PIDRatios::DOWN_SENSOR_BUFFER;

    bool are_inter_sensor_both_dark = sensor_values[1] > PIDRatios::DOWN_SENSOR_BUFFER && sensor_values[4] > PIDRatios::DOWN_SENSOR_BUFFER;
    bool are_outer_sensor_both_dark = sensor_values[0] > PIDRatios::DOWN_SENSOR_BUFFER && sensor_values[5] > PIDRatios::DOWN_SENSOR_BUFFER;

    if (is_all_white || is_all_dark || are_middle_sensors_dark || (are_middle_sensors_white && (are_inter_sensor_both_dark || are_outer_sensor_both_dark)) || (!are_middle_sensors_dark && !are_middle_sensors_white && are_middle_sensors_close)) {
        return true;
    }
    return false;
}

double RobotPID::getError(const std::vector<double> &sensor_values) {
    double error = 0;

    error += PIDRatios::INTER_MUL*(sensor_values[2] - sensor_values[3]);
    error += PIDRatios::MIDDLE_MUL*(sensor_values[1] - sensor_values[4]);
    error += PIDRatios::OUTER_MUL*(sensor_values[0] - sensor_values[5]);

    return error;
}