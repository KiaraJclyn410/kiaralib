#include "basic_motions.hpp"
#include "PID.hpp"
#include "helpers.hpp"

// Define the Lateral PID object for straight-line movement
// Tuning: kp (start small), ki (usually 0), kd (the brake), minSpeed, slew, maxSpeed
PID lateralPID(6.5, 0.0, 1.5, 0, 10, 127); 
PID angularPID(0.9, 0.0, 2.02, 0, 10, 127);

void driveInches(double target, double speed, double timeout, Odometry& odom, pros::MotorGroup& left_mg, pros::MotorGroup& right_mg) {
    uint32_t startTime = pros::millis();
    
    // 1. Reset the PID to clear old memory (integral/prevError)
    lateralPID.reset();
    lateralPID.maxSpeed = speed; // Set the speed limit for this specific move

    // 2. Record starting position
    double startY = odom.pose.y;
    double startX = odom.pose.x;

    while (pros::millis() - startTime < timeout) {
        // 3. Calculate current distance traveled
        // We use the Pythagorean distance formula in case the robot drifts slightly
        double currentX = odom.pose.x;
        double currentY = odom.pose.y;
        double distanceTraveled = std::sqrt(std::pow(currentX - startX, 2) + std::pow(currentY - startY, 2));
        
        // If driving backward, we make the distance negative
        if (target < 0) distanceTraveled *= -1;

        // 4. Calculate Error
        double error = target - distanceTraveled;

        // 5. Exit Condition: If we are within 0.5 inches of target
        if (std::abs(error) < 0.5) break;

        // 6. Compute Motor Power
        double power = lateralPID.compute(error);

        // 7. Output to motors
        left_mg.move(power);
        right_mg.move(power);

        pros::delay(10);
    }

    // 8. Stop motors
    left_mg.move(0);
    right_mg.move(0);
}

void turnToAngle(double targetAngle, double speed, double timeout, Odometry& odom, pros::MotorGroup& left_mg, pros::MotorGroup& right_mg) {
    uint32_t startTime = pros::millis();
    angularPID.reset();
    angularPID.maxSpeed = speed;

    double lastError = getTurnError(targetAngle, radToDeg(odom.pose.theta));

    while (pros::millis() - startTime < timeout) {
        double currentAngle = radToDeg(odom.pose.theta);
        double error = getTurnError(targetAngle, currentAngle);
        
        // Calculate velocity (change in error)
        double velocity = error - lastError; 
        lastError = error; // Update for the next loop

        double power = angularPID.compute(error);

        // --- VELOCITY EXIT CRITERIA ---
        // Exit if error is small AND the robot has physically slowed down
        if (std::abs(error) < 0.0 && std::abs(velocity) < 0.1) {
            pros::lcd::print(5, "Exited by criteria");
            break; 
        }

        left_mg.move(power);
        right_mg.move(-power);

        pros::lcd::print(1, "Deg: %.2f", currentAngle);
        pros::lcd::print(2, "Err: %.2f", error);
        pros::lcd::print(3, "Vel: %.2f", velocity);
        
        pros::delay(10);
    }

    left_mg.move(0);
    right_mg.move(0);
    pros::lcd::print(4, "Exited");
}