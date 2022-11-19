#pragma once
#include <gmock/gmock.h>

#include "analoginterface.h"

class AnalogMockInput : public AnalogInterface
{
public:
    AnalogMockInput() {}
    MOCK_METHOD(void, init, (), (override));
    MOCK_METHOD(double, getValue, (), (override));
};