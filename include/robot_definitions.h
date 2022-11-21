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

constexpr int SAFE_BUFFER = 200;

constexpr double MAX_SPEED = 100, MIN_SPEED = 40, STARTING_SPEED = 70.0;

enum class CALIBRATION_STATUS
{
    IDLE,
    WHITE,
    WAIT,
    BLACK
};

enum class ROBOT_STATUS
{
    UNCALIBRATED,
    CALIBRATION,
    READY,
    FOLLOWING
};

std::ostream &operator<<(std::ostream &os, const ROBOT_STATUS &status)
{
    switch (status)
    {
    case ROBOT_STATUS::UNCALIBRATED:
        os << "UNCALIBRATED";
        break;
    case ROBOT_STATUS::CALIBRATION:
        os << "CALIBRATION...";
        break;
    case ROBOT_STATUS::READY:
        os << "READY";
        break;
    case ROBOT_STATUS::FOLLOWING:
        os << "FOLLOWING";
        break;
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, const CALIBRATION_STATUS &status)
{
    switch (status)
    {
    case CALIBRATION_STATUS::IDLE:
        os << "IDLE";
        break;
    case CALIBRATION_STATUS::WHITE:
        os << "WHITE CALIBRATION";
        break;
    case CALIBRATION_STATUS::WAIT:
        os << "MOVE TO BLACK";
        break;
    case CALIBRATION_STATUS::BLACK:
        os << "BLACK CALIBRATION";
        break;
    }
    return os;
}