#include "sensor.h"

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
}

double Sensor::getBrightnessPercentage() {
    return -1;
}

double Sensor::getDenoisedValue() {
    return value;
}

void Sensor::setCalibrationState(SensorUtils::CalibrationState state) {

}