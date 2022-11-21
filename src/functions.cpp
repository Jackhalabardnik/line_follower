#include "functions.h"
#include "button.h"
#include "engine.h"
#include "sensor.h"
#include "analoginput.h"
#include "robotpid.cpp"

#include <sstream>
#include <iomanip>
#include <numeric>

namespace
{
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

	constexpr int MEAN_SIZE = 10;

	constexpr int DOWN_SENSOR_BUFFER = 10, UP_SENSOR_BUFFER = 90;

	constexpr int SAFE_BUFFER = 200;

	constexpr double MAX_SPEED = 100, MIN_SPEED = 40, IDLE_SPEED = 40.0;

	constexpr double MID_MUL = 0.2, INTER_MUL = 2;

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

	CALIBRATION_STATUS calibration_status = CALIBRATION_STATUS::IDLE;
	ROBOT_STATUS robot_status = ROBOT_STATUS::UNCALIBRATED;

	SSD1306Wire display(0x3c, SDA, SCL);

	std::vector<Sensor> sensor_board = {};

	RobotPID robotPID(IDLE_SPEED, MAX_SPEED, MIN_SPEED);

	Engine left_engine(FORWARD_LEFT_ENGINE_PIN, BACKWARD_LEFT_ENGINE_PIN, PWM_LEFT_ENGINE_PIN, LEFT_ENGINE_PWM_CHANNEL),
		right_engine(FORWARD_RIGHT_ENGINE_PIN, BACKWARD_RIGHT_ENGINE_PIN, PWM_RIGHT_ENGINE_PIN, RIGHT_ENGINE_PWM_CHANNEL);

	int left_engine_debug = 0;
	int right_engine_debug = 0;

	bool go_to_next_calibration_phase = false,
		 go_to_next_move_mode = false;

	int safe_buffer_value = 0;

	std::vector<Buttons::Button> buttons = {
		{CALIBRATION_PIN, []()
		 { go_to_next_calibration_phase = true; }},
		{ROBOT_STATUS_PIN, []()
		 { go_to_next_move_mode = true; }}};

	void init_OLED()
	{
		display.init();
		display.flipScreenVertically();
		display.setFont(ArialMT_Plain_10);
	}

	void init_adc()
	{
		sensor_board.emplace_back(std::make_unique<AnalogInput>(OUTER_LEFT_PIN));
		sensor_board.emplace_back(std::make_unique<AnalogInput>(INTER_LEFT_PIN));
		sensor_board.emplace_back(std::make_unique<AnalogInput>(MID_LEFT_PIN));
		sensor_board.emplace_back(std::make_unique<AnalogInput>(MID_RIGHT_PIN));
		sensor_board.emplace_back(std::make_unique<AnalogInput>(INTER_RIGHT_PIN));
		sensor_board.emplace_back(std::make_unique<AnalogInput>(OUTER_RIGHT_PIN));

		for(auto &sensor : sensor_board) {
			sensor.init();
		}
	}

	void init_engines()
	{
		left_engine.init();
		right_engine.init();
	}

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

	void start_following()
	{
		robot_status = ROBOT_STATUS::FOLLOWING;
	}

	void stop_following()
	{
		robot_status = ROBOT_STATUS::READY;
		left_engine.setSpeed(0);
		right_engine.setSpeed(0);
		left_engine_debug = 0;
		right_engine_debug = 0;
	}

	void check_safe_buffer() {
		if((sensor_board[2].getBlackPercentage() < DOWN_SENSOR_BUFFER && sensor_board[3].getBlackPercentage() < DOWN_SENSOR_BUFFER) 
		|| ((sensor_board[2].getBlackPercentage() > UP_SENSOR_BUFFER && sensor_board[3].getBlackPercentage() > UP_SENSOR_BUFFER))) {
			safe_buffer_value++;
		} else {
			safe_buffer_value = 0;
		}

		if(safe_buffer_value >= SAFE_BUFFER) {
			safe_buffer_value = 0;
			stop_following();
		}
	}
}

void init_devices()
{
	init_OLED();
	Serial.println("OLED done");
	init_adc();
	Serial.println("ADC done");
	init_engines();
	Serial.println("Engines screaming");
}

void refresh_screen()
{
	std::stringstream ss;

	if (robot_status == ROBOT_STATUS::CALIBRATION)
	{
		ss << calibration_status << "\n";
	}

	ss << "Status: " << robot_status << "\n";

	if (robot_status == ROBOT_STATUS::READY || robot_status == ROBOT_STATUS::FOLLOWING)
	{
		for (auto &sensor : sensor_board)
		{
			ss << std::round(sensor.getBlackPercentage()) << " ";
		}
		ss << "\nL: " << left_engine.getSpeed() << " R: " << right_engine.getSpeed() << "\n";
		ss << "\nLD: " << left_engine_debug << " RD: " << right_engine_debug << "\n";
	}

	display.clear();

	display.drawString(1, 1, ss.str().c_str());

	display.display();
}

void refresh_adc()
{
	for (auto &sensor : sensor_board)
	{
		sensor.measureBlackLevel();
	}
}

void refresh_buttons()
{
	for (auto &button : buttons)
	{
		button.updateButton();
	}
}

void do_ADC_calibration()
{
	if (go_to_next_calibration_phase)
	{
		auto calibrationState = SensorUtils::CalibrationState::NONE;
		switch (calibration_status)
		{
		case CALIBRATION_STATUS::IDLE:
			if (robot_status == ROBOT_STATUS::UNCALIBRATED || robot_status == ROBOT_STATUS::READY)
			{
				calibration_status = CALIBRATION_STATUS::WHITE;
				robot_status = ROBOT_STATUS::CALIBRATION;
				calibrationState = SensorUtils::CalibrationState::WHITE;
			}
			break;
		case CALIBRATION_STATUS::WHITE:
			calibration_status = CALIBRATION_STATUS::WAIT;
			break;
		case CALIBRATION_STATUS::WAIT:
			calibration_status = CALIBRATION_STATUS::BLACK;
			calibrationState = SensorUtils::CalibrationState::BLACK;
			break;
		case CALIBRATION_STATUS::BLACK:
			calibration_status = CALIBRATION_STATUS::IDLE;
			robot_status = ROBOT_STATUS::READY;
		}
		for (auto &sensor : sensor_board)
		{
			sensor.setCalibrationState(calibrationState);
		}

		go_to_next_calibration_phase = false;
	}
}

void check_move_mode()
{
	if (go_to_next_move_mode)
	{
		switch (robot_status)
		{
		case ROBOT_STATUS::UNCALIBRATED:
		case ROBOT_STATUS::CALIBRATION:
			break;
		case ROBOT_STATUS::READY:
			start_following();
			break;
		case ROBOT_STATUS::FOLLOWING:
			stop_following();
		}
		go_to_next_move_mode = false;
	}
}

void do_PID_calculation()
{
	std::vector<double> percentages;
	for(const auto &sensor: sensor_board) {
		percentages.emplace_back(sensor.getBlackPercentage());
	}
	auto velocities = robotPID.calculatePID(percentages);
	left_engine.setSpeed(velocities.leftEngineSpeed);
	right_engine.setSpeed(velocities.rightEngineSpeed);
}

void do_main_logic()
{
	do_ADC_calibration();
	check_move_mode();
	if (robot_status == ROBOT_STATUS::FOLLOWING)
	{
		do_PID_calculation();
		check_safe_buffer();
	}
}

void do_ADC_debug()
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