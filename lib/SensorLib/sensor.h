#pragma once
#include "analoginterface.h"

#include <list>
#include <memory>

namespace SensorUtils {

constexpr int MIN_SENSOR_VALUE = 0, MAX_SENSOR_VALUE = 4096, DOWN_SENSOR_BUFFER = 10, UP_SENSOR_BUFFER = 90, WINDOW_SIZE = 10;

enum class CalibrationState {NONE, WHITE, BLACK};

}

class Sensor
{
public:

    Sensor(std::unique_ptr<AnalogInterface> &&analog_input);

    void init();

    void measure_brightness();

    double get_brightness_percentage();

    double get_denoised_value();

    void set_calibration_state(SensorUtils::CalibrationState state);

private:
    double value = 0, min = SensorUtils::MIN_SENSOR_VALUE, max = SensorUtils::MAX_SENSOR_VALUE, percentage = 0;
    std::list<double> values;
    std::unique_ptr<AnalogInterface> analog_input;
    SensorUtils::CalibrationState calibration_state;
};