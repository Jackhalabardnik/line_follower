#include "functions.h"
#include "periodic_execution.h"

std::vector<PeriodicExecution::Routine> routines;
constexpr int main_logic_time = 10,
              screen_refresh_time = 33, // 30 FPS
              adc_refresh_time = 1,
              button_refresh_time = 2;

void setup() {
  Serial.begin(9600);
  init_devices();

  routines.push_back({adc_refresh_time, refresh_adc});
  routines.push_back({button_refresh_time, refresh_buttons});
  routines.push_back({main_logic_time, do_main_logic});
  routines.push_back({screen_refresh_time, refresh_screen});
}

void loop() {
  while(true) {
    PeriodicExecution::updateExecutions(routines);
  }
}