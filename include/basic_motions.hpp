#pragma once
#include "main.h"
#include "PID.hpp"
#include "odometry.hpp"

/**
 * Drive a specific distance in inches using the vertical tracking wheel.
 * * @param target The distance to drive (positive for forward, negative for back)
 * @param speed The maximum power allowed (0-127)
 * @param timeout The maximum time allowed for the move in milliseconds
 */
void driveInches(double target, double speed, double timeout, Odometry& odom, pros::MotorGroup& left_mg, pros::MotorGroup& right_mg);
void turnToAngle(double targetAngle, double speed, double timeout, Odometry& odom, pros::MotorGroup& left_mg, pros::MotorGroup& right_mg);