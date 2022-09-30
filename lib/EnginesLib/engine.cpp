#include "engine.h"

Engine::Engine(int forward_pin, int backward_pin, int pwm_pin, int pwm_channel)
    : forward_pin(forward_pin), backward_pin(backward_pin), pwm_pin(pwm_pin), pwm_channel(pwm_channel), pwm_level(0)
{
}

void Engine::init()
{
    pinMode(forward_pin, OUTPUT);
    digitalWrite(forward_pin, 0);

    pinMode(backward_pin, OUTPUT);
    digitalWrite(backward_pin, 0);

    ledcAttachPin(pwm_pin, pwm_channel);
    ledcSetup(pwm_channel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcWrite(pwm_channel, pwm_level);
}

void Engine::set_speed(double pwm_percentage)
{
    pwm_percentage = pwm_percentage < 0 ? 0 : pwm_percentage;
    pwm_percentage = pwm_percentage > 100 ? 100 : pwm_percentage;

    if (pwm_level == 0 && pwm_percentage > 0)
    {
        digitalWrite(forward_pin, 1);
    }
    else if (pwm_level > 0 && pwm_percentage == 0)
    {
        digitalWrite(forward_pin, 0);
    }

    pwm_level = (pwm_percentage / 100.0) * MAX_PWM_VALUE;

    ledcWrite(pwm_channel, pwm_level);
}

int Engine::get_speed()
{
    return double(pwm_level) / MAX_PWM_VALUE * 100;
}