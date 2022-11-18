#pragma once
#include "analoginterface.h"

class AnalogInput : public AnalogInterface
{
    AnalogInput(int pin_number);

    void init() override;
    double get_value() override;
private:
    int analog_pin_number;
};