#include "robotpid.h"
#include "utils.h"

#include <math.h>

RobotPID::RobotPID(double maxEngineSpeed, double minEngineSpeed)
    : maxEngineSpeed(maxEngineSpeed),
      minEngineSpeed(minEngineSpeed){}

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
    // PIDSkipped = false;

    double current_error = 0;

    if (PIDSkipped) {
        current_error = lastError;
    } else {
        current_error = getError(sensor_values);
    }

    double min_speed = std::min(lastEngineSpeed.leftEngineSpeed, lastEngineSpeed.rightEngineSpeed);
    double engine_mul = (1 - PIDRatios::ENGINE_MUL) * ((min_speed - minEngineSpeed) / (maxEngineSpeed - minEngineSpeed) * PIDRatios::ENGINE_MUL);

    engine_mul = 1;

    proportionalPart = current_error * PIDRatios::PROPORTIONAL_MUL * engine_mul;
    integralPart += current_error * PIDRatios::INTEGRAL_MUL * engine_mul;
    derivativePart = (current_error - lastError) * PIDRatios::DERIVATIVE_MUL * engine_mul;
    lastError = current_error;

    double sum = proportionalPart + integralPart + derivativePart;

    double leftEngineSpeed = maxEngineSpeed;
    double rightEngineSpeed = maxEngineSpeed;
    
    // CHANGE ENGINE MUUUUUUUUUUUUUUUUUUUUUUUUUL
    if(sum < 0) {
        leftEngineSpeed += maxEngineSpeed * (sum/maxEngineSpeed);
    }
    if(sum > 0) {
        rightEngineSpeed -= maxEngineSpeed * (sum/maxEngineSpeed);
    }

    bound_value(leftEngineSpeed, minEngineSpeed, maxEngineSpeed);
    bound_value(rightEngineSpeed, minEngineSpeed, maxEngineSpeed);

    lastEngineSpeed = {leftEngineSpeed, rightEngineSpeed};

    return lastEngineSpeed;
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

    if(is_all_white || is_all_dark) {
        return true; // But this is a subject to a change in the future
    }

    bool inter_right = sensor_values[4] > PIDRatios::DOWN_SENSOR_BUFFER;
    bool inter_left = sensor_values[1] > PIDRatios::DOWN_SENSOR_BUFFER;
    bool outer_right = sensor_values[5] > PIDRatios::DOWN_SENSOR_BUFFER;
    bool outer_left = sensor_values[0] > PIDRatios::DOWN_SENSOR_BUFFER;

    if((outer_left || inter_left) && (inter_right || outer_right)) {
        return true;
    }
    return false;
}

double RobotPID::getError(const std::vector<double> &sensor_values) {
    double error = 0;

    if(sensor_values[0] > PIDRatios::DOWN_SENSOR_BUFFER && lastError < PIDRatios::MIDDLE_MUL) {
        return -PIDRatios::OUTER_MUL;
    }
    if(sensor_values[5] > PIDRatios::DOWN_SENSOR_BUFFER && lastError > PIDRatios::MIDDLE_MUL) {
        return PIDRatios::OUTER_MUL;
    }

    if(sensor_values[1] > PIDRatios::DOWN_SENSOR_BUFFER && lastError < 0) {
        return -PIDRatios::INTER_MUL;
    }
    if(sensor_values[4] > PIDRatios::DOWN_SENSOR_BUFFER && lastError > 0) {
        return PIDRatios::INTER_MUL;
    }

    if(sensor_values[2] > 60 && sensor_values[3] > 60) {
        return 0;
    }

    if(sensor_values[2] > 0 && sensor_values[3] > 0) {
        return PIDRatios::MIDDLE_MUL*(sensor_values[3] - sensor_values[2])/100;
    }

    if(sensor_values[2] > PIDRatios::DOWN_SENSOR_BUFFER) {
        error = -PIDRatios::MIDDLE_MUL;
    }
    if(sensor_values[3] > PIDRatios::DOWN_SENSOR_BUFFER) {
        error = PIDRatios::MIDDLE_MUL;
    }

    return error;
}

double RobotPID::getLastError() {
    return lastError;
}