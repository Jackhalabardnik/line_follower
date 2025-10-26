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
    analog_value = analogInput->getValue();

    if(abs(analog_value - current_sensor_value) > SensorUtils::IIR_threshold) {
        current_sensor_value = analog_value;
    } else {
        current_sensor_value = current_sensor_value * SensorUtils::IIR_alpha + analog_value * (1.0 - SensorUtils::IIR_alpha);
    }

    if(current_sensor_value > max) {
        max = current_sensor_value;
    }

    if(current_sensor_value < min) {
        min = current_sensor_value;
    }
}

double Sensor::getBlackPercentage() const {
    double dead_zone = (max - min) * 0.25;

    if (dead_zone < SensorUtils::IIR_threshold) {
        dead_zone = SensorUtils::IIR_threshold;
    }

    double percentage = 0;
    if (max - current_sensor_value < dead_zone) {
        percentage = 0;
    } else if (current_sensor_value - min < dead_zone) {
        percentage = 100;
    } else {
        percentage = 100.0 - ((current_sensor_value - min) / (max - min)) * 100.0;
    }
    bound_value(percentage, 0.0, 100.0);
    return percentage;
}

double Sensor::getDenoisedValue() const {
    return current_sensor_value;
}

double Sensor::getRawValue() const {
    return analog_value;
}

void Sensor::setCalibrationState(SensorUtils::CalibrationState state) {
    calibrationState = state;
    if (state == SensorUtils::CalibrationState::WHITE) {
        max = SensorUtils::MAX_SENSOR_VALUE;
    } else if (state == SensorUtils::CalibrationState::BLACK) {
        min = SensorUtils::MIN_SENSOR_VALUE;
    }
}