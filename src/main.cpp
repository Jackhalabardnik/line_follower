#include "functions.h"
#include "periodic_execution.h"

std::vector<PeriodicExecution::Routine> routines;
constexpr int screen_refresh_time = 50,
              adc_refresh_time = 2;

void setup() {
  Serial.begin(9600);
  init_devices();

  routines.push_back({screen_refresh_time, refresh_screen});
  routines.push_back({adc_refresh_time, refresh_adc});
}

void loop() {
  while(true) {
    PeriodicExecution::updateExecutions(routines);
  }
}