#include "functions.h"
#include "periodic_execution.h"

// #define LINE_SENSOR_DEBUG_MODE

std::vector<PeriodicExecution::Routine> routines;
constexpr int adcRefreshTime = 1,
              buttonRefreshTime = 2,
              mainLogicTime = 10,
              screenRefreshTime = 33;// 30 FPS

void setup() {
    Serial.begin(9600);
    Serial.println("\n");
    Serial.println("Begin setup");
    initDevices();

    routines.push_back({adcRefreshTime, refreshAdc});
    routines.push_back({buttonRefreshTime, refreshButtons});
    routines.push_back({mainLogicTime, doMainLogic});
    routines.push_back({screenRefreshTime, refreshScreen});
    Serial.println("Finished setup");
}

void loop() {
    while (true) {
        #ifdef LINE_SENSOR_DEBUG_MODE
            debugADC();
        #else
            PeriodicExecution::updateExecutions(routines);
        #endif
    }
}