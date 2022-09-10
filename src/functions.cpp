#include "functions.h"
#include <sstream>

namespace
{
	constexpr int OUTER_LEFT_PIN = A19,
				  INTER_LEFT_PIN = A18,
				  MID_LEFT_PIN = A5,
				  MID_RIGHT_PIN = A4,
				  INTER_RIGHT_PIN = A7,
				  OUTER_RIGHT_PIN = A6;

	SSD1306Wire display(0x3c, SDA, SCL);

	double adc_values[6];

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
	ss <<  adc_values[0]
	   << " " << adc_values[1]
	   << " " << adc_values[2]
	   << "\n" << adc_values[3]
	   << " " << adc_values[4]
	   << " " << adc_values[5];

	display.clear();

	display.drawString(1, 1, ss.str().c_str());

	display.display();
}

void refresh_adc()
{
	adc_values[0] = analogRead(OUTER_LEFT_PIN);
	adc_values[1] = analogRead(INTER_LEFT_PIN);
	adc_values[2] = analogRead(MID_LEFT_PIN);
	adc_values[3] = analogRead(MID_RIGHT_PIN);
	adc_values[4] = analogRead(INTER_RIGHT_PIN);
	adc_values[5] = analogRead(OUTER_RIGHT_PIN);
}