#pragma once
#include <Arduino.h>
#include <vector>
#include <functional>

namespace Buttons
{
    class Button
    {
    public:
        Button(int pin_number, std::function<void()> fall_callback);

        void update_button();

        bool pressed();

    private:
        const int pin;
        int score = 0;
        bool is_activated = false;
        bool has_changed = false;

        std::function<void()> fall_callback;
    };

    // activation level: false - button is pressed when logic 0, true - button is
    // pressed when logic 1
    constexpr bool activation_level = false;

    constexpr int button_max_score = 10, button_score_threshold = button_max_score / 2;
} // namespace Buttons
