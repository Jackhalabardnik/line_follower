#include "sensor.h"

Sensor::Sensor(const int pin, std::unique_ptr<AnalogInterface> analog_input)
    : pin(pin), analog_input(std::move(analog_input)) {}