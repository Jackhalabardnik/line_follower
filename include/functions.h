#pragma once
#include <Arduino.h>

#include <SSD1306Wire.h>

void init_devices();

void refresh_screen();

void refresh_adc();

void refresh_buttons();

void do_ADC_calibration();

void do_ADC_debug();