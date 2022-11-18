#pragma once

#include <random>

using distribution = std::uniform_int_distribution<std::mt19937::result_type>;

std::random_device dev;
std::mt19937 rng(dev());