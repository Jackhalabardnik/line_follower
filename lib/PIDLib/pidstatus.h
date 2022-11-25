#include <iostream>

enum class PIDStatus {
    PROPORTIONAL,
    LIGHT_CURVE_LEFT,
    LIGHT_CURVE_RIGHT,
    HEAVY_CURVE_LEFT,
    HEAVY_CURVE_RIGHT
};


std::ostream &operator<<(std::ostream &os, const PIDStatus &status)
{
    switch (status)
    {
    case PIDStatus::PROPORTIONAL:
        os << "PROPORTIONAL";
        break;
    case PIDStatus::LIGHT_CURVE_LEFT:
        os << "LIGHT_CURVE_LEFT";
        break;
    case PIDStatus::LIGHT_CURVE_RIGHT:
        os << "LIGHT_CURVE_RIGHT";
        break;
    case PIDStatus::HEAVY_CURVE_LEFT:
        os << "HEAVY_CURVE_LEFT";
        break;
    case PIDStatus::HEAVY_CURVE_RIGHT:
        os << "HEAVY_CURVE_RIGHT";
        break;
    }
    return os;
}