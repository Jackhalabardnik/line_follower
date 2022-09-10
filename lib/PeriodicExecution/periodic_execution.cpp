#include "periodic_execution.h"

namespace PeriodicExecution {
    Routine::Routine(unsigned long _period, void (*_function)()) {
        period = _period;
        function = _function;
    }

    void updateExecutions(std::vector<Routine> &routines) {
        std::for_each(routines.begin(), routines.end(), [](Routine &routine) {
            if (millis() - routine.last_execution_time > routine.period || routine.last_execution_time > millis()) {
                routine.function();
                routine.last_execution_time = millis();
            }
        });
    }
}