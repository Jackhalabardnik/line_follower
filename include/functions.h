#pragma once
#include <Arduino.h>

#include <SSD1306Wire.h>
#include "robotpid.h"

void initDevices();

void refreshAdc();

void refreshButtons();

void doMainLogic();

void refreshScreen();

void checkRobotStatus();

RobotEngineSpeed calculatePID();

void setEngineSpeed(RobotEngineSpeed speed);

void debugADC();