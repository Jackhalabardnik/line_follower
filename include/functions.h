#pragma once
#include <Arduino.h>

#include <SSD1306Wire.h>

void init_devices();

void refresh_screen();

void refresh_adc();

void refresh_buttons();

void do_ADC_calibration();

void check_move_mode();

void do_PID_calculation();

void set_engine_speed();

void do_main_logic();

void do_ADC_debug();