#include "analoginput.h"

AnalogInput::AnalogInput(int pinNumber)
    : analogPinNumber(pinNumber) {}

void AnalogInput::init() {
    pinMode(analogPinNumber, INPUT);
}

double AnalogInput::getValue() {
    return analogRead(analogPinNumber);
}