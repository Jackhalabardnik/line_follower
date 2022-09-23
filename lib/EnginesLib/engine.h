#include "Arduino.h"

constexpr int PWM_RESOLUTION = 10, PWM_FREQUENCY = 1000, MAX_PWM_VALUE = 1 << PWM_RESOLUTION;

class Engine
{
public:
    Engine(int power_pin, int direction_pin, int pwm_pin, int pwm_channel);

    void set_speed(int pwm_speed);

    int get_speed();

private:
    const int power_pin, direction_pin, pwm_pin, pwm_channel;
    int pwm_level;
};