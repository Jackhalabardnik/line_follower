#pragma once
#include <Arduino.h>

#include <SSD1306Wire.h>

void initDevices();

void refreshAdc();

void refreshButtons();

void doMainLogic();

void refreshScreen();

void checkCalibrationStatus();

void checkRobotStatus();

void calculatePID();

void debugADC();