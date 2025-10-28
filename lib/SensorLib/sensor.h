#pragma once
#include "analoginterface.h"

#include <list>
#include <memory>

namespace SensorUtils {

    constexpr int MIN_SENSOR_VALUE = 0, MAX_SENSOR_VALUE = 4096, WINDOW_SIZE = 10,
                  IIR_THRESHOLD = 150, NULL_VALUE = -1, IIR_JUMPS_THRESHOLD = 2;

    constexpr double IIR_ALPHA = 0.95;

    enum class CalibrationState { NONE,
                                  WHITE,
                                  BLACK };

}// namespace SensorUtils

class Sensor {
public:
    Sensor(std::unique_ptr<AnalogInterface> &&analogInput);

    void init();

    void measureBlackLevel();

    double getBlackPercentage() const;

    double getDenoisedValue() const;

    double getRawValue() const;

    void setCalibrationState(SensorUtils::CalibrationState state);

private:
    double current_sensor_value = SensorUtils::NULL_VALUE, min = SensorUtils::MAX_SENSOR_VALUE,
           max = SensorUtils::MIN_SENSOR_VALUE, analog_value = 0;
    int iir_jumps_registered = 0;
    std::unique_ptr<AnalogInterface> analogInput;
    SensorUtils::CalibrationState calibrationState;
};