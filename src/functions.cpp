#include "functions.h"
#include "button.h"
#include "engine.h"
#include <sstream>
#include <iomanip>
#include <list>
#include <vector>
#include <numeric>

namespace
{
	constexpr int OUTER_LEFT_PIN = A5,
				  INTER_LEFT_PIN = A4,
				  MID_LEFT_PIN = A7,
				  MID_RIGHT_PIN = A6,
				  INTER_RIGHT_PIN = A3,
				  OUTER_RIGHT_PIN = A0,
				  CALIBRATION_PIN = GPIO_NUM_25,
				  ROBOT_STATUS_PIN = GPIO_NUM_26,
				  PWM_LEFT_ENGINE_PIN = GPIO_NUM_19,
				  FORWARD_LEFT_ENGINE_PIN = GPIO_NUM_18,
				  BACKWARD_LEFT_ENGINE_PIN = GPIO_NUM_17,
				  BACKWARD_RIGHT_ENGINE_PIN = GPIO_NUM_16,
				  FORWARD_RIGHT_ENGINE_PIN = GPIO_NUM_27,
				  PWM_RIGHT_ENGINE_PIN = GPIO_NUM_14;

	constexpr int LEFT_ENGINE_PWM_CHANNEL = 1,
				  RIGHT_ENGINE_PWM_CHANNEL = 2;

	constexpr int MEAN_SIZE = 10;

	constexpr int MIN_SENSOR_VALUE = 0, MAX_SENSOR_VALUE = 4096, SENSOR_BUFFER = 10;

	constexpr int SAFE_BUFFER = 200;

	constexpr double MAX_SPEED = 100.0, MIN_SPEED = 0, IDLE_SPEED = 60.0;

	constexpr double MID_MUL = 1, INTER_MUL = 3;

	struct Sensor
	{
		Sensor(const int _pin, int mean = 0, std::list<double> values = {}) : pin(_pin) {}
		const int pin;
		double value = 0, min = MIN_SENSOR_VALUE, max = MAX_SENSOR_VALUE, percentage = 0;
		std::list<double> values = {};
	};

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

	std::vector<Sensor> sensor_board = {
		{OUTER_LEFT_PIN},
		{INTER_LEFT_PIN},
		{MID_LEFT_PIN},
		{MID_RIGHT_PIN},
		{INTER_RIGHT_PIN},
		{OUTER_RIGHT_PIN},
	};

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
		pinMode(OUTER_LEFT_PIN, INPUT);
		pinMode(INTER_LEFT_PIN, INPUT);
		pinMode(MID_LEFT_PIN, INPUT);
		pinMode(MID_RIGHT_PIN, INPUT);
		pinMode(INTER_RIGHT_PIN, INPUT);
		pinMode(OUTER_RIGHT_PIN, INPUT);
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
		left_engine.set_speed(0);
		right_engine.set_speed(0);
		left_engine_debug = 0;
		right_engine_debug = 0;
	}

	void check_safe_buffer() {
		if(sensor_board[2].percentage < SENSOR_BUFFER && sensor_board[3].percentage < SENSOR_BUFFER) {
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
	init_adc();
	init_engines();
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
		for (const auto &sensor : sensor_board)
		{
			ss << std::round(sensor.percentage) << " ";
		}
		ss << "\nL: " << left_engine.get_speed() << " R: " << right_engine.get_speed() << "\n";
		ss << "\nLD: " << left_engine_debug << " RD: " << right_engine_debug << "\n";
	}

	display.clear();

	display.drawString(1, 1, ss.str().c_str());

	// if (robot_status == ROBOT_STATUS::READY || robot_status == ROBOT_STATUS::FOLLOWING)
	// {
	// 	display.fillRect(1, 40, left_engine.get_speed(), 5);
	// 	display.fillRect(1, 50, right_engine.get_speed(), 5);
	// }

	display.display();
}

void refresh_adc()
{
	for (auto &sensor : sensor_board)
	{
		sensor.values.emplace_back(analogRead(sensor.pin));
		if (sensor.values.size() > MEAN_SIZE)
		{
			sensor.values.pop_front();
		}
		sensor.value = std::round(*std::max_element(sensor.values.begin(), sensor.values.end()) / 10);
		if (calibration_status == CALIBRATION_STATUS::WHITE && sensor.value < sensor.max)
		{
			sensor.max = sensor.value;
		}
		if (calibration_status == CALIBRATION_STATUS::BLACK && sensor.value > sensor.min)
		{
			sensor.min = sensor.value;
		}

		sensor.percentage = (sensor.value - sensor.min) / (sensor.max - sensor.min);
		sensor.percentage = sensor.percentage > 1 ? 1 : sensor.percentage;
		sensor.percentage = sensor.percentage < 0 ? 0 : sensor.percentage;
		sensor.percentage *= 100.0;
		sensor.percentage = 100.0 - sensor.percentage;
	}
}

void refresh_buttons()
{
	for (auto &button : buttons)
	{
		button.update_button();
	}
}

void do_ADC_calibration()
{
	if (go_to_next_calibration_phase)
	{
		switch (calibration_status)
		{
		case CALIBRATION_STATUS::IDLE:
			if (robot_status == ROBOT_STATUS::UNCALIBRATED || robot_status == ROBOT_STATUS::READY)
			{
				calibration_status = CALIBRATION_STATUS::WHITE;
				robot_status = ROBOT_STATUS::CALIBRATION;

				for (auto &sensor : sensor_board)
				{
					sensor.min = MIN_SENSOR_VALUE;
					sensor.max = MAX_SENSOR_VALUE;
				}
			}
			break;
		case CALIBRATION_STATUS::WHITE:
			calibration_status = CALIBRATION_STATUS::WAIT;
			break;
		case CALIBRATION_STATUS::WAIT:
			calibration_status = CALIBRATION_STATUS::BLACK;
			break;
		case CALIBRATION_STATUS::BLACK:
			calibration_status = CALIBRATION_STATUS::IDLE;
			robot_status = ROBOT_STATUS::READY;
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
	double left_engine_speed = IDLE_SPEED, right_engine_speed = IDLE_SPEED;

	double equilibrium = (sensor_board[2].percentage + sensor_board[3].percentage) / 2;

	left_engine_speed += (sensor_board[2].percentage - equilibrium) * MID_MUL;
	right_engine_speed += (sensor_board[3].percentage - equilibrium) * MID_MUL;

	if (sensor_board[1].percentage > SENSOR_BUFFER)
	{
		left_engine_speed -= sensor_board[1].percentage * INTER_MUL;
		right_engine_speed += sensor_board[1].percentage * INTER_MUL;
	}

	if (sensor_board[4].percentage > SENSOR_BUFFER)
	{
		left_engine_speed += sensor_board[4].percentage * INTER_MUL;
		right_engine_speed -= sensor_board[4].percentage * INTER_MUL;
	}

	left_engine_debug = left_engine_speed;
	right_engine_debug = right_engine_speed;

	left_engine.set_speed(left_engine_speed);
	right_engine.set_speed(right_engine_speed);
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