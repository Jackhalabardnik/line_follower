#include "robotpid.h"
#include "utils.h"

#include <math.h>

RobotPID::RobotPID(double startEngineSpeed, double maxEngineSpeed, double minEngineSpeed)
    : maxEngineSpeed(maxEngineSpeed),
      minEngineSpeed(minEngineSpeed),
      idleEngineSpeed(startEngineSpeed) {}

PIDParts RobotPID::getPIDStatus()
{
    return {proportionalPart, integralPart, derivativePart};
}

bool RobotPID::isPIDSkipped() {
    return PIDSkipped;
}

void RobotPID::resetPID() {
    integralPart = 0;
    derivativePart = 0;
}

RobotEngineSpeed RobotPID::calculatePID(const std::vector<double> &sensor_values)
{
    PIDSkipped = needToSkipPID(sensor_values);

    double error = getError(sensor_values);

    double lastIntegralPart = integralPart;
    if(PIDSkipped) {
        proportionalPart = 0;
    }   else {
        proportionalPart = error * PIDRatios::PROPORTIONAL_MUL;
        integralPart += error * PIDRatios::INTEGRAL_MUL;
        derivativePart = (lastIntegralPart - integralPart) * PIDRatios::DERIVATIVE_MUL;
    }

    double sum = proportionalPart + integralPart + derivativePart;

    double leftEngineSpeed = idleEngineSpeed + sum;
    double rightEngineSpeed = idleEngineSpeed - sum;

    bound_value(leftEngineSpeed, minEngineSpeed, maxEngineSpeed);
    bound_value(rightEngineSpeed, minEngineSpeed, maxEngineSpeed);

    return {leftEngineSpeed, rightEngineSpeed};
}

bool RobotPID::needToSkipPID(const std::vector<double> &sensor_values)
{
    bool is_all_white = true, is_all_dark = true;

    for (const auto &sensor_value : sensor_values)
    {
        if (sensor_value > PIDRatios::DOWN_SENSOR_BUFFER)
        {
            is_all_white = false;
        }
        if (sensor_value < PIDRatios::UP_SENSOR_BUFFER)
        {
            is_all_dark = false;
        }
    }

    bool are_middle_sensors_dark = sensor_values[2] > PIDRatios::UP_SENSOR_BUFFER && sensor_values[3] > PIDRatios::UP_SENSOR_BUFFER;
    bool are_middle_sensors_white = sensor_values[2] < PIDRatios::DOWN_SENSOR_BUFFER && sensor_values[3] < PIDRatios::DOWN_SENSOR_BUFFER;
    bool are_middle_sensors_close = std::abs(sensor_values[2] - sensor_values[3]) < PIDRatios::DOWN_SENSOR_BUFFER;

    bool are_inter_sensor_both_dark = sensor_values[1] > PIDRatios::DOWN_SENSOR_BUFFER && sensor_values[4] > PIDRatios::DOWN_SENSOR_BUFFER;
    bool are_outer_sensor_both_dark = sensor_values[0] > PIDRatios::DOWN_SENSOR_BUFFER && sensor_values[5] > PIDRatios::DOWN_SENSOR_BUFFER;

    if (is_all_white || is_all_dark || are_middle_sensors_dark || (are_middle_sensors_white && (are_inter_sensor_both_dark || are_outer_sensor_both_dark)) || (!are_middle_sensors_dark && !are_middle_sensors_white && are_middle_sensors_close))
    {
        return true;
    }
    return false;
}

double RobotPID::getError(const std::vector<double> &sensor_values)
{
    double error = 0;

    if (sensor_values[2] > PIDRatios::DOWN_SENSOR_BUFFER || sensor_values[3] > PIDRatios::DOWN_SENSOR_BUFFER)
    {
        if (sensor_values[2] > PIDRatios::DOWN_SENSOR_BUFFER && sensor_values[3] > PIDRatios::DOWN_SENSOR_BUFFER)
        {
            error += (sensor_values[2] + sensor_values[3]) / 2;
        }
        else
        {
            error += sensor_values[2] > PIDRatios::DOWN_SENSOR_BUFFER ? sensor_values[2] : sensor_values[3];
        }

        if (sensor_values[2] > sensor_values[3])
        {
            error *= -1;
        }
        error *= PIDRatios::MIDDLE_MUL;
    }
    if (sensor_values[1] >= PIDRatios::DOWN_SENSOR_BUFFER)
    {
        error += sensor_values[1] * PIDRatios::INTER_MUL * -1;
    }
    else if (sensor_values[4] >= PIDRatios::DOWN_SENSOR_BUFFER)
    {
        error += sensor_values[4] * PIDRatios::INTER_MUL;
    }
    if (sensor_values[0] >= PIDRatios::DOWN_SENSOR_BUFFER)
    {
        error += sensor_values[0] * PIDRatios::OUTER_MUL * -1;
    }
    else if (sensor_values[5] >= PIDRatios::DOWN_SENSOR_BUFFER)
    {
        error += sensor_values[5] * PIDRatios::OUTER_MUL;
    }

    return error;
}