#include "Arduino.h"

constexpr int PWM_RESOLUTION = 10, PWM_FREQUENCY = 1000, MAX_PWM_VALUE = 1 << PWM_RESOLUTION;

class Engine
{
public:
    Engine(int forward_pin, int backward_pin, int pwm_pin, int pwm_channel);

    void init();

    void set_speed(double pwm_speed);

    int get_speed();

private:
    const int forward_pin, backward_pin, pwm_pin, pwm_channel;
    int pwm_level;
};