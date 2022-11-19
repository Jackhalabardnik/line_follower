#include "periodic_execution.h"

namespace PeriodicExecution {
    Routine::Routine(unsigned long _period, void (*_function)()) {
        period = _period;
        function = _function;
    }

    void updateExecutions(std::vector<Routine> &routines) {
        std::for_each(routines.begin(), routines.end(), [](Routine &routine) {
            if (millis() - routine.lastExecutionTime > routine.period || routine.lastExecutionTime > millis()) {
                routine.function();
                routine.lastExecutionTime = millis();
            }
        });
    }
}