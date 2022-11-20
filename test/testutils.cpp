#include "testutils.h"

double generate_random_number(int right_cap, int left_cap) {
    std::random_device dev;
    std::mt19937 randomGenerator(dev());
    std::uniform_int_distribution<std::mt19937::result_type> distribution(right_cap, left_cap);
    return distribution(randomGenerator);
}