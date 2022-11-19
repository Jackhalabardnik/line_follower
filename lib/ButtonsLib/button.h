#pragma once
#include <Arduino.h>
#include <vector>
#include <functional>

namespace Buttons
{
    class Button
    {
    public:
        Button(int pin_number, std::function<void()> fallCallback);

        void updateButton();

        bool pressed();

    private:
        const int pin;
        int score = 0;
        bool isActivated = false;
        bool hasChanged = false;

        std::function<void()> fallCallback;
    };

    // activation level: false - button is pressed when logic 0, true - button is
    // pressed when logic 1
    constexpr bool activationLevel = true;

    constexpr int buttonMaxScore = 10, buttonScoreThreshold = buttonMaxScore / 2;
} // namespace Buttons
