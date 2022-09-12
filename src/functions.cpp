#include "functions.h"
#include <sstream>
#include <list>
#include <vector>
#include <numeric>

namespace
{
	struct Sensor
	{
		Sensor(const int _pin, int mean = 0, std::list<double> values = {}) : pin(_pin) {}
		const int pin;
		double mean = 0;
		std::list<double> values = {};
	};

	constexpr int OUTER_LEFT_PIN = A19,
				  INTER_LEFT_PIN = A18,
				  MID_LEFT_PIN = A5,
				  MID_RIGHT_PIN = A4,
				  INTER_RIGHT_PIN = A7,
				  OUTER_RIGHT_PIN = A6;
	constexpr int MEAN_SIZE = 10;

	SSD1306Wire display(0x3c, SDA, SCL);

	std::vector<Sensor> sensor_board = {
		{OUTER_LEFT_PIN},
		{INTER_LEFT_PIN},
		{MID_LEFT_PIN},
		{MID_RIGHT_PIN},
		{INTER_RIGHT_PIN},
		{OUTER_RIGHT_PIN},
	};

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
		if (i++ == 3)
		{
			ss << "\n";
		}
		ss << sensor.mean << " ";
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
		sensor.mean = std::round(*std::max_element(sensor.values.begin(), sensor.values.end())/100);
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