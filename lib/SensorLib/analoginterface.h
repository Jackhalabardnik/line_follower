#pragma once

class AnalogInterface
{
public:
    virtual void init() = 0;
    virtual double get_value() = 0;
    virtual ~AnalogInterface() {};
};