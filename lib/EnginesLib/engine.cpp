#include "engine.h"

Engine::Engine(int power_pin, int direction_pin, int pwm_pin, int pwm_channel)
    : power_pin(power_pin), direction_pin(direction_pin), pwm_pin(pwm_pin), pwm_channel(pwm_channel), pwm_level(0)
{
    pinMode(power_pin, OUTPUT);
    digitalWrite(power_pin, 0);

    pinMode(direction_pin, OUTPUT);
    digitalWrite(direction_pin, 0);

    ledcAttachPin(pwm_pin, pwm_channel);
    ledcSetup(pwm_channel, PWM_FREQUENCY, PWM_RESOLUTION);
}

void Engine::set_speed(int pwm_speed)
{
    pwm_speed = pwm_speed < 0 ? 0 : pwm_speed;
    pwm_speed = pwm_speed > MAX_PWM_VALUE ? MAX_PWM_VALUE : pwm_speed;

    if (pwm_level == 0 && pwm_speed > 0)
    {
        digitalWrite(power_pin, 1);
    }
    else if (pwm_level > 0 && pwm_speed == 0)
    {
        digitalWrite(power_pin, 0);
    }

    pwm_level = pwm_speed;

    ledcWrite(pwm_channel, pwm_level);
}

int Engine::get_speed()
{
    return pwm_level;
}