#include "functions.h"
#include "analoginput.h"
#include "button.h"
#include "engine.h"
#include "robot_definitions.h"
#include "robotpid.cpp"
#include "sensor.h"

#include <numeric>
#include <sstream>

namespace {
    CalibrationStatus calibrationStatus = CalibrationStatus::IDLE;
    RobotStatus robotStatus = RobotStatus::UNCALIBRATED;

    SSD1306Wire display(0x3c, SDA, SCL);

    std::vector<Sensor> sensorBoard = {};

    RobotPID robotPID(STARTING_SPEED, MAX_SPEED, MIN_SPEED);

    Engine leftEngine(FORWARD_LEFT_ENGINE_PIN, BACKWARD_LEFT_ENGINE_PIN, PWM_LEFT_ENGINE_PIN, LEFT_ENGINE_PWM_CHANNEL),
            rightEngine(FORWARD_RIGHT_ENGINE_PIN, BACKWARD_RIGHT_ENGINE_PIN, PWM_RIGHT_ENGINE_PIN, RIGHT_ENGINE_PWM_CHANNEL);

    int safeBufferValue = 0;

    bool goToNextCalibrationPhase = false,
         goToNextMoveMode = false;

    std::vector<Buttons::Button> buttons = {
            {CALIBRATION_PIN, []() { goToNextCalibrationPhase = true; }},
            {ROBOT_STATUS_PIN, []() { goToNextMoveMode = true; }}};

    void initOLED() {
        display.init();
        display.flipScreenVertically();
        display.setFont(ArialMT_Plain_10);
    }

    void initADC() {
        sensorBoard.emplace_back(std::make_unique<AnalogInput>(OUTER_LEFT_PIN));
        sensorBoard.emplace_back(std::make_unique<AnalogInput>(INTER_LEFT_PIN));
        sensorBoard.emplace_back(std::make_unique<AnalogInput>(MID_LEFT_PIN));
        sensorBoard.emplace_back(std::make_unique<AnalogInput>(MID_RIGHT_PIN));
        sensorBoard.emplace_back(std::make_unique<AnalogInput>(INTER_RIGHT_PIN));
        sensorBoard.emplace_back(std::make_unique<AnalogInput>(OUTER_RIGHT_PIN));

        for (auto &sensor: sensorBoard) {
            sensor.init();
        }
    }

    void initEngines() {
        leftEngine.init();
        rightEngine.init();
    }

    void startFollowing() {
        robotStatus = RobotStatus::FOLLOWING;
        robotPID.resetPID();
    }

    void stopFollowing() {
        robotStatus = RobotStatus::READY;
        leftEngine.setSpeed(0);
        rightEngine.setSpeed(0);
    }

    void checkSafeBuffer() {
        if (robotPID.isPIDSkipped()) {
            safeBufferValue++;
        } else {
            safeBufferValue = 0;
        }

        if (safeBufferValue >= SAFE_BUFFER) {
            safeBufferValue = 0;
            stopFollowing();
        }
    }
}// namespace

void initDevices() {
    initOLED();
    Serial.println("OLED done");
    initADC();
    Serial.println("ADC done");
    initEngines();
    Serial.println("Engines screaming");
}

void refreshAdc() {
    for (auto &sensor: sensorBoard) {
        sensor.measureBlackLevel();
    }
}

void refreshButtons() {
    for (auto &button: buttons) {
        button.updateButton();
    }
}

void doMainLogic() {
    checkCalibrationStatus();
    checkRobotStatus();
    if (robotStatus == RobotStatus::FOLLOWING) {
        calculatePID();
        checkSafeBuffer();
    }
}

void refreshScreen() {
    std::stringstream ss;

    if (robotStatus == RobotStatus::CALIBRATION) {
        ss << calibrationStatus << "\n";
    }

    ss << "Status: " << robotStatus << "\n";

    if (robotStatus == RobotStatus::READY || robotStatus == RobotStatus::FOLLOWING) {
        for (auto &sensor: sensorBoard) {
            ss << std::round(sensor.getBlackPercentage()) << " ";
        }
        ss << "\nL: " << leftEngine.getSpeed() << " R: " << rightEngine.getSpeed() << "\n";
        auto parts = robotPID.getPIDStatus();
        ss << "P: " << std::round(parts.proportional) << " I: " << parts.intergal << " D: " << parts.derivative << "\n";
    }

    display.clear();

    display.drawString(1, 1, ss.str().c_str());

    display.display();
}

void checkCalibrationStatus() {
    if (goToNextCalibrationPhase) {
        auto calibrationState = SensorUtils::CalibrationState::NONE;
        switch (calibrationStatus) {
            case CalibrationStatus::IDLE:
                if (robotStatus == RobotStatus::UNCALIBRATED || robotStatus == RobotStatus::READY) {
                    calibrationStatus = CalibrationStatus::WHITE;
                    robotStatus = RobotStatus::CALIBRATION;
                    calibrationState = SensorUtils::CalibrationState::WHITE;
                }
                break;
            case CalibrationStatus::WHITE:
                calibrationStatus = CalibrationStatus::WAIT;
                break;
            case CalibrationStatus::WAIT:
                calibrationStatus = CalibrationStatus::BLACK;
                calibrationState = SensorUtils::CalibrationState::BLACK;
                break;
            case CalibrationStatus::BLACK:
                calibrationStatus = CalibrationStatus::IDLE;
                robotStatus = RobotStatus::READY;
        }

        for (auto &sensor: sensorBoard) {
            sensor.setCalibrationState(calibrationState);
        }

        goToNextCalibrationPhase = false;
    }
}

void checkRobotStatus() {
    if (goToNextMoveMode) {
        switch (robotStatus) {
            case RobotStatus::UNCALIBRATED:
            case RobotStatus::CALIBRATION:
                break;
            case RobotStatus::READY:
                startFollowing();
                break;
            case RobotStatus::FOLLOWING:
                stopFollowing();
        }
        goToNextMoveMode = false;
    }
}

void calculatePID() {
    std::vector<double> percentages;
    for (const auto &sensor: sensorBoard) {
        percentages.emplace_back(sensor.getBlackPercentage());
    }

    auto velocities = robotPID.calculatePID(percentages);

    leftEngine.setSpeed(velocities.leftEngineSpeed);
    rightEngine.setSpeed(velocities.rightEngineSpeed);
}

void debugADC() {
    int samples[] = {OUTER_LEFT_PIN, INTER_LEFT_PIN ,MID_LEFT_PIN ,MID_RIGHT_PIN ,INTER_RIGHT_PIN ,OUTER_RIGHT_PIN};
    std::string raw_data = "";
    for (auto pin: samples) {
        double value = analogRead(pin);
        raw_data += std::to_string(value) + ",";
    }

    Serial.println(raw_data.c_str());
}