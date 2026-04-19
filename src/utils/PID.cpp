#include "PID.hpp"
#include <cmath>
#include <algorithm>

PID::PID(double kp, double ki, double kd, double minSpeed, double slew, double max)
    : kp(kp), ki(ki), kd(kd), minSpeed(minSpeed), slewRate(slew), maxSpeed(max) {}

double PID::compute(double error) {
    //PID calculation
    double p = kp * error;
    integral += error;
    double i = ki * integral;
    double d = kd * (error - prevError);
    prevError = error;

    double output = p + i + d;

    //Slew Rate (Accel Limiting)
    double diff = output - prevOutput;
    if (std::abs(diff) > slewRate) {
        output = prevOutput + (diff > 0 ? slewRate : -slewRate);
    }

    //Min Speed
    if (std::abs(output) < minSpeed) {
        output = (output > 0) ? minSpeed : -minSpeed;
    }

    //output constraints
    output = std::clamp(output, -maxSpeed, maxSpeed);
    
    prevOutput = output;
    return output;
}
//reset
void PID::reset() {
    prevError = 0;
    prevOutput = 0;
    integral = 0;
}