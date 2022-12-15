#pragma once

#include <Arduino.h>
#include <sstream>

constexpr int OUTER_LEFT_PIN = A5,
              INTER_LEFT_PIN = A4,
              MID_LEFT_PIN = A7,
              MID_RIGHT_PIN = A6,
              INTER_RIGHT_PIN = A3,
              OUTER_RIGHT_PIN = A0,
              CALIBRATION_PIN = GPIO_NUM_19,
              ROBOT_STATUS_PIN = GPIO_NUM_18,
              PWM_LEFT_ENGINE_PIN = GPIO_NUM_16,
              FORWARD_LEFT_ENGINE_PIN = GPIO_NUM_26,
              BACKWARD_LEFT_ENGINE_PIN = GPIO_NUM_17,
              BACKWARD_RIGHT_ENGINE_PIN = GPIO_NUM_27,
              FORWARD_RIGHT_ENGINE_PIN = GPIO_NUM_14,
              PWM_RIGHT_ENGINE_PIN = GPIO_NUM_25;

constexpr int LEFT_ENGINE_PWM_CHANNEL = 1,
              RIGHT_ENGINE_PWM_CHANNEL = 2;

constexpr int SAFE_BUFFER = 400;

constexpr double MAX_SPEED = 100.0, MIN_SPEED = 0, STARTING_SPEED = 50.0;

enum class CalibrationStatus {
    IDLE,
    WHITE,
    WAIT,
    BLACK
};

enum class RobotStatus {
    UNCALIBRATED,
    CALIBRATION,
    READY,
    FOLLOWING
};

std::ostream &operator<<(std::ostream &os, const RobotStatus &status) {
    switch (status) {
        case RobotStatus::UNCALIBRATED:
            os << "UNCALIBRATED";
            break;
        case RobotStatus::CALIBRATION:
            os << "CALIBRATION...";
            break;
        case RobotStatus::READY:
            os << "READY";
            break;
        case RobotStatus::FOLLOWING:
            os << "FOLLOWING";
            break;
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const CalibrationStatus &status) {
    switch (status) {
        case CalibrationStatus::IDLE:
            os << "IDLE";
            break;
        case CalibrationStatus::WHITE:
            os << "WHITE CALIBRATION";
            break;
        case CalibrationStatus::WAIT:
            os << "MOVE TO BLACK";
            break;
        case CalibrationStatus::BLACK:
            os << "BLACK CALIBRATION";
            break;
    }
    return os;
}