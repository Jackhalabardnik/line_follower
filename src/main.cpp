#include "functions.h"
#include "periodic_execution.h"

std::vector<PeriodicExecution::Routine> routines;
constexpr int adcRefreshTime = 1, buttonRefreshTime = 2, mainLogicTime = 10,
              screenRefreshTime = 33; // 30 FPS

void setup() {
  Serial.begin(9600);
  Serial.println("\n");
  Serial.println("Begin setup");
  init_devices();

  routines.push_back({adcRefreshTime, refresh_adc});
  routines.push_back({buttonRefreshTime, refresh_buttons});
  routines.push_back({mainLogicTime, do_main_logic});
  routines.push_back({screenRefreshTime, refresh_screen});
  Serial.println("Finished setup");
}

void loop() {
  while (true) {
    PeriodicExecution::updateExecutions(routines);
  }
}