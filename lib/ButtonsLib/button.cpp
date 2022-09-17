#include "button.h"

namespace Buttons
{

    Button::Button(int pin_number, std::function<void()> fall_callback)
        : pin(pin_number), fall_callback(fall_callback)
    {
        pinMode(pin, INPUT);
    }

    void Button::update_button()
    {
        bool is_pin_active = (digitalRead(pin) == activation_level);

        if (is_pin_active && score < button_max_score)
        {
            score++;
        }
        if (!is_pin_active && score > 0)
        {
            score--;
        }

        if (is_activated && score < button_score_threshold)
        {
            is_activated = false;
            has_changed = true;
            fall_callback();
        }
        else if (score == button_max_score)
        {
            is_activated = true;
            has_changed = true;
        }
    }

    bool Button::pressed() {
        return is_activated;
    }
}