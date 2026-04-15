#include "main.h"
#include "helpers.hpp"


double angleError(double a, double b) {
    double diff = a - b;
    while (diff > M_PI) diff -= 2*M_PI;
    while (diff < -M_PI) diff += 2*M_PI;
    return diff;
}