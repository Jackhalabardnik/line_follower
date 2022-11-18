#include "sensor.h"

#include <algorithm>
#include <cmath>

Sensor::Sensor(std::unique_ptr<AnalogInterface> &&analog_input)
    : analog_input(std::move(analog_input)) {}

void Sensor::init() {
    analog_input->init();
}

void Sensor::measure_brightness() {
    values.emplace_back(analog_input->get_value());
    value = *std::max_element(values.begin(), values.end());
}

double Sensor::get_brightness_percentage() {
    return -1;
}

double Sensor::get_denoised_value() {
    return value;
}

void Sensor::set_calibration_state(CalibrationState state) {

}