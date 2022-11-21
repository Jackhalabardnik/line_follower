#pragma once
#include "analoginterface.h"

#include <list>
#include <memory>

namespace SensorUtils {

constexpr int MIN_SENSOR_VALUE = 0, MAX_SENSOR_VALUE = 4096, WINDOW_SIZE = 10;

enum class CalibrationState {NONE, WHITE, BLACK};

}

class Sensor
{
public:

    Sensor(std::unique_ptr<AnalogInterface> &&analogInput);

    void init();

    void measureBlackLevel();

    double getBlackPercentage() const;

    double getDenoisedValue() const;

    void setCalibrationState(SensorUtils::CalibrationState state);

private:
    double value = 0, min = SensorUtils::MIN_SENSOR_VALUE, max = SensorUtils::MAX_SENSOR_VALUE, percentage = 0;
    std::list<double> values;
    std::unique_ptr<AnalogInterface> analogInput;
    SensorUtils::CalibrationState calibrationState;
};