#pragma once

class AnalogInterface {
public:
    virtual void init() = 0;
    virtual double getValue() = 0;
    virtual ~AnalogInterface(){};
};