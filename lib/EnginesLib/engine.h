#include "Arduino.h"

constexpr int PWM_RESOLUTION = 10, PWM_FREQUENCY = 1000, MAX_PWM_VALUE = 1 << PWM_RESOLUTION;

class Engine
{
public:
    Engine(int forwardPin, int backwardPin, int pwmPin, int pwmChannel);

    void init();

    void setSpeed(double pwmPercentage);

    int getSpeed();

private:
    const int forwardPin, backwardPin, pwmPin, pwmChannel;
    int pwmLevel;
};