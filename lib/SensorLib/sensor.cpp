#include "sensor.h"
#include "utils.h"

#include <algorithm>
#include <cmath>

Sensor::Sensor(std::unique_ptr<AnalogInterface> &&analogInput)
    : analogInput(std::move(analogInput)) {}

void Sensor::init() {
    analogInput->init();
}

void Sensor::measureBlackLevel() {
    values.emplace_back(analogInput->getValue());
    if(values.size() > SensorUtils::WINDOW_SIZE) {
        values.pop_front();
    }
    value = *std::max_element(values.begin(), values.end());

    if(calibrationState == SensorUtils::CalibrationState::WHITE && value < max) {
        max = value;
    } 

    if(calibrationState == SensorUtils::CalibrationState::BLACK && value > min) {
        min = value;
    }

    if((calibrationState == SensorUtils::CalibrationState::WHITE || calibrationState == SensorUtils::CalibrationState::BLACK) && min == max) {
        max = SensorUtils::MAX_SENSOR_VALUE;
        min = SensorUtils::MIN_SENSOR_VALUE;
    } 

}

double Sensor::getBlackPercentage() {
    auto percentage = 100.0 - ((value - min) / (max - min)) * 100.0;
    bound_value(percentage, 0.0, 100.0);
    return percentage;
}

double Sensor::getDenoisedValue() {
    return value;
}

void Sensor::setCalibrationState(SensorUtils::CalibrationState state) {
    calibrationState = state;
    if(state == SensorUtils::CalibrationState::WHITE) {
        max = SensorUtils::MAX_SENSOR_VALUE;
    } else if(state == SensorUtils::CalibrationState::BLACK) {
        min = SensorUtils::MIN_SENSOR_VALUE;
    }
}