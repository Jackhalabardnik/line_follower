#pragma once
#include "analoginterface.h"

class AnalogInput : public AnalogInterface
{
    AnalogInput(int pinNumber);

    void init() override;
    double getValue() override;
private:
    int analogPinNumber;
};