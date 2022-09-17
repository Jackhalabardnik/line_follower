#include "functions.h"
#include "button.h"
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
				  OUTER_RIGHT_PIN = A0;
	constexpr int MEAN_SIZE = 10;

	constexpr int MIN_SENSOR_VALUE = 0, MAX_SENSOR_VALUE = 4096;

	struct Sensor
	{
		Sensor(const int _pin, int mean = 0, std::list<double> values = {}) : pin(_pin) {}
		const int pin;
		double value = 0, min = 0, max = 4096, percentage = 0;
		std::list<double> values = {};
	};

	enum class CALIBRATION_STATUS {
		IDLE,
		WHITE, 
		WAIT,
		BLACK
	};

	CALIBRATION_STATUS calibration_status = CALIBRATION_STATUS::IDLE;

	SSD1306Wire display(0x3c, SDA, SCL);

	std::vector<Sensor> sensor_board = {
		{OUTER_LEFT_PIN},
		{INTER_LEFT_PIN},
		{MID_LEFT_PIN},
		{MID_RIGHT_PIN},
		{INTER_RIGHT_PIN},
		{OUTER_RIGHT_PIN},
	};

	bool go_to_next_phase = false;

	std::vector<Buttons::Button> buttons = {
		{GPIO_NUM_26, []()
		 { go_to_next_phase = true; }}};

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
}

void init_devices()
{
	init_OLED();
	init_adc();
}

void refresh_screen()
{
	std::stringstream ss;
	int i = 0;
	for (const auto &sensor : sensor_board)
	{
		ss << sensor.value << " " << sensor.min << " " << sensor.max;
		if (i++ % 2)
		{
			ss << "\n";
		} else {
			ss << " | ";
		}
	}

	ss << "\n" << int(calibration_status) << "\n";

	for (const auto &sensor : sensor_board)
	{
		ss << std::round(sensor.percentage) << " ";
	}

	display.clear();

	display.drawString(1, 1, ss.str().c_str());

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
		if(calibration_status == CALIBRATION_STATUS::WHITE && sensor.value < sensor.max) {
			sensor.max = sensor.value;
		}
		if(calibration_status == CALIBRATION_STATUS::BLACK && sensor.value > sensor.min) {
			sensor.min = sensor.value;
		}

		sensor.percentage = (sensor.value - sensor.min)/(sensor.max - sensor.min);
		sensor.percentage = sensor.percentage > 1 ? 1 : sensor.percentage;
		sensor.percentage = sensor.percentage < 0 ? 0 : sensor.percentage;
		sensor.percentage *= 100;
	}
}

void refresh_buttons()
{
	for (auto &button : buttons)
	{
		button.update_button();
	}
}

void do_ADC_calibration() {
	switch (calibration_status)
	{
	case CALIBRATION_STATUS::IDLE:
		if(go_to_next_phase) {
			calibration_status = CALIBRATION_STATUS::WHITE;

			for(auto &sensor: sensor_board) {
				sensor.min = MIN_SENSOR_VALUE;
				sensor.max = MAX_SENSOR_VALUE;
			}

			go_to_next_phase = false;
		}
		break;
	case CALIBRATION_STATUS::WHITE:
		if(go_to_next_phase) {
			calibration_status = CALIBRATION_STATUS::WAIT;
			go_to_next_phase = false;
		}
		break;
	case CALIBRATION_STATUS::WAIT:
		if(go_to_next_phase) {
			calibration_status = CALIBRATION_STATUS::BLACK;
			go_to_next_phase = false;
		}
		break;
	case CALIBRATION_STATUS::BLACK:
		if(go_to_next_phase) {
			calibration_status = CALIBRATION_STATUS::IDLE;
			go_to_next_phase = false;
		}
		break;
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