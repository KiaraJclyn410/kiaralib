#pragma once

class PID {
public:
    // Constants
    double kp, ki, kd;
    double minSpeed; // Used for motion chaining (kick-start/maintain speed)
    double slewRate; // Max change in power per loop
    double maxSpeed; 

    // State Variables
    double prevError = 0;
    double prevOutput = 0;
    double integral = 0;

    // Constructor
    PID(double kp, double ki, double kd, double minSpeed = 0, double slew = 127, double max = 127);

    // Compute method
    double compute(double error);

    // Reset for new movements
    void reset();
};