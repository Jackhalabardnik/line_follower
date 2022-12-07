#include "engine.h"

Engine::Engine(int forwardPin, int backwardPin, int pwmPin, int pwmChannel)
    : forwardPin(forwardPin), backwardPin(backwardPin), pwmPin(pwmPin), pwmChannel(pwmChannel), pwmLevel(0) {
}

void Engine::init() {
    pinMode(forwardPin, OUTPUT);
    digitalWrite(forwardPin, 0);

    pinMode(backwardPin, OUTPUT);
    digitalWrite(backwardPin, 0);

    ledcAttachPin(pwmPin, pwmChannel);
    ledcSetup(pwmChannel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcWrite(pwmChannel, pwmLevel);
}

void Engine::setSpeed(double pwmPercentage) {
    pwmPercentage = pwmPercentage < 0 ? 0 : pwmPercentage;
    pwmPercentage = pwmPercentage > 100 ? 100 : pwmPercentage;

    if (pwmLevel == 0 && pwmPercentage > 0) {
        digitalWrite(forwardPin, 1);
    } else if (pwmLevel > 0 && pwmPercentage == 0) {
        digitalWrite(forwardPin, 0);
    }

    pwmLevel = (pwmPercentage / 100.0) * MAX_PWM_VALUE;

    ledcWrite(pwmChannel, pwmLevel);
}

int Engine::getSpeed() {
    return double(pwmLevel) / MAX_PWM_VALUE * 100;
}