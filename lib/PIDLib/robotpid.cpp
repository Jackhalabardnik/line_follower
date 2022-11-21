#include "robotpid.h"
#include "utils.h"

#include <math.h>

RobotPID::RobotPID(double startEngineSpeed, double maxEngineSpeed, double minEngineSpeed)
    : maxEngineSpeed(maxEngineSpeed),
      minEngineSpeed(minEngineSpeed),
      leftEngineSpeed(startEngineSpeed),
      rightEngineSpeed(startEngineSpeed) {}

RobotEngineSpeed RobotPID::calculatePID(std::vector<double> sensor_values)
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

    if (is_all_white 
        || is_all_dark 
        || are_middle_sensors_dark 
        || (are_middle_sensors_white && (are_inter_sensor_both_dark || are_outer_sensor_both_dark)) 
        || (!are_middle_sensors_dark && !are_middle_sensors_white && are_middle_sensors_close))
    {
        return {leftEngineSpeed, rightEngineSpeed};
    }

    double error = 0;
    double proportional_multiplier = 0;

    if (sensor_values[2] > 0 || sensor_values[3] > 0)
    {
        error = (sensor_values[2] + sensor_values[3]) / 2;
        proportional_multiplier = PIDRatios::PROPORTIONAL_MID_MUL;
        if (sensor_values[2] > sensor_values[3])
        {
            error *= -1;
        }
    }
    else
    {
        if (sensor_values[1] >= PIDRatios::DOWN_SENSOR_BUFFER)
        {
            error = sensor_values[1];
            proportional_multiplier = PIDRatios::PROPORTIONAL_INTER_MUL * -1;
        }
        else if (sensor_values[4] >= PIDRatios::DOWN_SENSOR_BUFFER)
        {
            error = sensor_values[4];
            proportional_multiplier = PIDRatios::PROPORTIONAL_INTER_MUL;
        }
        else if (sensor_values[0] >= PIDRatios::DOWN_SENSOR_BUFFER)
        {
            error = sensor_values[0];
            proportional_multiplier = PIDRatios::PROPORTIONAL_OUTER_MUL * -1;
        }
        else if (sensor_values[5] >= PIDRatios::DOWN_SENSOR_BUFFER)
        {
            error = sensor_values[5];
            proportional_multiplier = PIDRatios::PROPORTIONAL_OUTER_MUL;
        }
    }

    double proportional_part = error * proportional_multiplier;
    leftIntegralPart += error;
    rightIntegralPart -= error;

    leftEngineSpeed += proportional_part + (leftIntegralPart*PIDRatios::INTEGRAL_MUL);
    rightEngineSpeed -= proportional_part + (rightIntegralPart*PIDRatios::INTEGRAL_MUL);

    bound_value(leftEngineSpeed, minEngineSpeed, maxEngineSpeed);
    bound_value(rightEngineSpeed, minEngineSpeed, maxEngineSpeed);

    return {leftEngineSpeed, rightEngineSpeed};
}