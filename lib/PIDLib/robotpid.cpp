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

    double proportional = 0;

    if (sensor_values[2] > 0 || sensor_values[3] > 0)
    {
        proportional = ((sensor_values[2] + sensor_values[3]) / 2) * PIDRatios::MID_MUL;
        if (sensor_values[2] > sensor_values[3])
        {
            proportional *= -1;
        }
    }
    else
    {
        if (sensor_values[0] >= PIDRatios::DOWN_SENSOR_BUFFER)
        {
            proportional = sensor_values[0] * PIDRatios::OUTER_MUL * -1;
        }
        else if (sensor_values[1] >= PIDRatios::DOWN_SENSOR_BUFFER)
        {
            proportional = sensor_values[1] * PIDRatios::INTER_MUL * -1;
        }
        else if (sensor_values[4] >= PIDRatios::DOWN_SENSOR_BUFFER)
        {
            proportional = sensor_values[4] * PIDRatios::INTER_MUL;
        }
        else if (sensor_values[5] >= PIDRatios::DOWN_SENSOR_BUFFER)
        {
            proportional = sensor_values[5] * PIDRatios::OUTER_MUL;
        }
    }

    leftEngineSpeed += proportional;
    rightEngineSpeed -= proportional;

    bound_value(leftEngineSpeed, minEngineSpeed, maxEngineSpeed);
    bound_value(rightEngineSpeed, minEngineSpeed, maxEngineSpeed);

    return {leftEngineSpeed, rightEngineSpeed};
}