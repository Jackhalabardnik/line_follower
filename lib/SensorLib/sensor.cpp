#include "sensor.h"
#include "utils.h"

#include <algorithm>
#include <cmath>

Sensor::Sensor(std::unique_ptr<AnalogInterface> &&analogInput)
    : analogInput(std::move(analogInput)) {}

void Sensor::init() {
    analogInput->init();
}

void Sensor::measureBrightness() {
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

}

double Sensor::getBrightnessPercentage() {
    auto percentage = 100.0 - ((value - min) / (max - min)) * 100.0;
    bound_value(percentage, 0.0, 100.0);
    return percentage;
}

double Sensor::getDenoisedValue() {
    return value;
}

void Sensor::setCalibrationState(SensorUtils::CalibrationState state) {
    calibrationState = state;
}