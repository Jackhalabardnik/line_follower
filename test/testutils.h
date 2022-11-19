#pragma once

#include <random>

using randomDistribution = std::uniform_int_distribution<std::mt19937::result_type>;

std::random_device dev;
std::mt19937 randomGenerator(dev());