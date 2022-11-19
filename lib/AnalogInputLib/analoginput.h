#pragma once
#include "analoginterface.h"
#include <Arduino.h>

class AnalogInput : public AnalogInterface
{
    AnalogInput(int pinNumber);

    void init() override;
    double getValue() override;
private:
    int analogPinNumber;
};