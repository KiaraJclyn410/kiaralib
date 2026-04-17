#include "main.h"
#include "helpers.hpp"


double angleError(double target, double current) {
    double error = target - current;
    // Use fmod to wrap instead of while loops
    error = fmod(error + M_PI, 2 * M_PI);
    if (error <= 0) error += 2 * M_PI;
    return error - M_PI;
}