#include "button.h"

namespace Buttons
{

    Button::Button(int pinNumber, std::function<void()> fallCallback)
        : pin(pinNumber), fallCallback(fallCallback)
    {
        pinMode(pin, INPUT);
    }

    void Button::updateButton()
    {
        bool isPinActive = (digitalRead(pin) == activationLevel);

        if (isPinActive && score < buttonMaxScore)
        {
            score++;
        }
        if (!isPinActive && score > 0)
        {
            score--;
        }

        if (isActivated && score < buttonScoreThreshold)
        {
            isActivated = false;
            hasChanged = true;
            fallCallback();
        }
        else if (score == buttonMaxScore)
        {
            isActivated = true;
            hasChanged = true;
        }
    }

    bool Button::pressed() {
        return isActivated;
    }
}