#pragma once
#include <Arduino.h>
#include <algorithm>
#include <vector>

namespace PeriodicExecution {
    struct Routine {
        Routine(unsigned long _period, void (*_function)()); 

        unsigned long period = 0;

        void (*function)() = nullptr;

        unsigned long lastExecutionTime = 0;
    };

    void updateExecutions(std::vector<Routine> &routines);
}