#include "functions.h"
#include "periodic_execution.h"

std::vector<PeriodicExecution::Routine> routines;
constexpr int screen_refresh_time = 20,
              adc_refresh_time = 1,
              button_refresh_time = 5;

void setup() {
  Serial.begin(9600);
  init_devices();

  routines.push_back({screen_refresh_time, refresh_screen});
  routines.push_back({adc_refresh_time, refresh_adc});
  routines.push_back({button_refresh_time, refresh_buttons});
  routines.push_back({screen_refresh_time, do_ADC_calibration});
}

void loop() {
  while(true) {
    PeriodicExecution::updateExecutions(routines);
  }
}