#include "functions.h"
#include "button.h"
#include "engine.h"
#include "sensor.h"
#include "analoginput.h"
#include "robotpid.cpp"
#include "robot_definitions.h"

#include <sstream>
#include <numeric>

namespace
{
	CALIBRATION_STATUS calibrationStatus = CALIBRATION_STATUS::IDLE;
	ROBOT_STATUS robotStatus = ROBOT_STATUS::UNCALIBRATED;

	SSD1306Wire display(0x3c, SDA, SCL);

	std::vector<Sensor> sensorBoard = {};

	RobotPID robotPID(STARTING_SPEED, MAX_SPEED, MIN_SPEED);

	Engine leftEngine(FORWARD_LEFT_ENGINE_PIN, BACKWARD_LEFT_ENGINE_PIN, PWM_LEFT_ENGINE_PIN, LEFT_ENGINE_PWM_CHANNEL),
		rightEngine(FORWARD_RIGHT_ENGINE_PIN, BACKWARD_RIGHT_ENGINE_PIN, PWM_RIGHT_ENGINE_PIN, RIGHT_ENGINE_PWM_CHANNEL);

	int safeBufferValue = 0;

	bool goToNextCalibrationPhase = false,
		 goToNextMoveMode = false;

	std::vector<Buttons::Button> buttons = {
		{CALIBRATION_PIN, []()
		 { goToNextCalibrationPhase = true; }},
		{ROBOT_STATUS_PIN, []()
		 { goToNextMoveMode = true; }}};

	void initOLED()
	{
		display.init();
		display.flipScreenVertically();
		display.setFont(ArialMT_Plain_10);
	}

	void initADC()
	{
		sensorBoard.emplace_back(std::make_unique<AnalogInput>(OUTER_LEFT_PIN));
		sensorBoard.emplace_back(std::make_unique<AnalogInput>(INTER_LEFT_PIN));
		sensorBoard.emplace_back(std::make_unique<AnalogInput>(MID_LEFT_PIN));
		sensorBoard.emplace_back(std::make_unique<AnalogInput>(MID_RIGHT_PIN));
		sensorBoard.emplace_back(std::make_unique<AnalogInput>(INTER_RIGHT_PIN));
		sensorBoard.emplace_back(std::make_unique<AnalogInput>(OUTER_RIGHT_PIN));

		for(auto &sensor : sensorBoard) {
			sensor.init();
		}
	}

	void initEngines()
	{
		leftEngine.init();
		rightEngine.init();
	}

	void startFollowing()
	{
		robotStatus = ROBOT_STATUS::FOLLOWING;
	}

	void stopFollowing()
	{
		robotStatus = ROBOT_STATUS::READY;
		leftEngine.setSpeed(0);
		rightEngine.setSpeed(0);
	}

	void checkSafeBuffer() {
		if((sensorBoard[2].getBlackPercentage() < PIDRatios::DOWN_SENSOR_BUFFER && sensorBoard[3].getBlackPercentage() < PIDRatios::DOWN_SENSOR_BUFFER) 
		|| (sensorBoard[2].getBlackPercentage() > PIDRatios::UP_SENSOR_BUFFER && sensorBoard[3].getBlackPercentage() > PIDRatios::UP_SENSOR_BUFFER)) {
			safeBufferValue++;
		} else {
			safeBufferValue = 0;
		}

		if(safeBufferValue >= SAFE_BUFFER) {
			safeBufferValue = 0;
			stopFollowing();
		}
	}
}

void initDevices()
{
	initOLED();
	Serial.println("OLED done");
	initADC();
	Serial.println("ADC done");
	initEngines();
	Serial.println("Engines screaming");
}

void refreshAdc()
{
	for (auto &sensor : sensorBoard)
	{
		sensor.measureBlackLevel();
	}
}

void refreshButtons()
{
	for (auto &button : buttons)
	{
		button.updateButton();
	}
}

void doMainLogic()
{
	checkCalibrationStatus();
	checkRobotStatus();
	if (robotStatus == ROBOT_STATUS::FOLLOWING)
	{
		calculatePID();
		checkSafeBuffer();
	}
}

void refreshScreen()
{
	std::stringstream ss;

	if (robotStatus == ROBOT_STATUS::CALIBRATION)
	{
		ss << calibrationStatus << "\n";
	}

	ss << "Status: " << robotStatus << "\n";

	if (robotStatus == ROBOT_STATUS::READY || robotStatus == ROBOT_STATUS::FOLLOWING)
	{
		for (auto &sensor : sensorBoard)
		{
			ss << std::round(sensor.getBlackPercentage()) << " ";
		}
		ss << "\nL: " << leftEngine.getSpeed() << " R: " << rightEngine.getSpeed() << "\n";
	}

	display.clear();

	display.drawString(1, 1, ss.str().c_str());

	display.display();
}

void checkCalibrationStatus()
{
	if (goToNextCalibrationPhase)
	{
		auto calibrationState = SensorUtils::CalibrationState::NONE;
		switch (calibrationStatus)
		{
		case CALIBRATION_STATUS::IDLE:
			if (robotStatus == ROBOT_STATUS::UNCALIBRATED || robotStatus == ROBOT_STATUS::READY)
			{
				calibrationStatus = CALIBRATION_STATUS::WHITE;
				robotStatus = ROBOT_STATUS::CALIBRATION;
				calibrationState = SensorUtils::CalibrationState::WHITE;
			}
			break;
		case CALIBRATION_STATUS::WHITE:
			calibrationStatus = CALIBRATION_STATUS::WAIT;
			break;
		case CALIBRATION_STATUS::WAIT:
			calibrationStatus = CALIBRATION_STATUS::BLACK;
			calibrationState = SensorUtils::CalibrationState::BLACK;
			break;
		case CALIBRATION_STATUS::BLACK:
			calibrationStatus = CALIBRATION_STATUS::IDLE;
			robotStatus = ROBOT_STATUS::READY;
		}
		
		for (auto &sensor : sensorBoard)
		{
			sensor.setCalibrationState(calibrationState);
		}

		goToNextCalibrationPhase = false;
	}
}

void checkRobotStatus()
{
	if (goToNextMoveMode)
	{
		switch (robotStatus)
		{
		case ROBOT_STATUS::UNCALIBRATED:
		case ROBOT_STATUS::CALIBRATION:
			break;
		case ROBOT_STATUS::READY:
			startFollowing();
			break;
		case ROBOT_STATUS::FOLLOWING:
			stopFollowing();
		}
		goToNextMoveMode = false;
	}
}

void calculatePID()
{
	std::vector<double> percentages;
	for(const auto &sensor: sensorBoard) {
		percentages.emplace_back(sensor.getBlackPercentage());
	}
	
	auto velocities = robotPID.calculatePID(percentages);

	leftEngine.setSpeed(velocities.leftEngineSpeed);
	rightEngine.setSpeed(velocities.rightEngineSpeed);
}

void debugADC()
{
	constexpr int sampling_period = 10000;

	short *values = new short[sampling_period];

	for (int i = 0; i < sampling_period; i++)
	{
		values[i] = analogRead(INTER_LEFT_PIN);
		delay(1);
	}

	Serial.print("#############################\n");

	for (int i = 0; i < sampling_period; i++)
	{
		Serial.println(values[i]);
	}

	Serial.print("#############################\n");
}