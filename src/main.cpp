#include "main.h"
#include "pros/misc.hpp"
#include "pros/rotation.hpp"
#include "odometry.hpp"
#include "pros/rtos.hpp"
#include "basic_motions.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({-18, -19, -20});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({13, 12, 11});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

pros::Rotation vEnc(16);
pros::Rotation hEnc(17);
pros::Imu imu(15);

TrackingWheel verticalWheel(&vEnc, 2.75, -0.1575, 1);
TrackingWheel horizontalWheel(&hEnc, 2.75, -2.875, 1);

Odometry odom(&verticalWheel, &horizontalWheel, &imu);

 void odom_task_fn() {
    while (true) {
        odom.update();
        // Run faster than the main loop for better precision
        pros::delay(10); 
    }
}


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	imu.reset();  // Resets the IMU's heading to zero
	pros::delay(2000); 
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	pros::Task odom_task(odom_task_fn);
	
	turnToAngle(90, 127, 80000, odom, left_mg,right_mg);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void opcontrol() {
	
	odom.setPose(0, 0,  0);


	pros::Task odom_task(odom_task_fn);

    while (true) {
        // Driver code only handles JOYSTICKS and MOTORS
        int dir = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        
        left_mg.move(dir + turn);
        right_mg.move(dir - turn);

        // Just read the pose, don't update it here
        pros::lcd::print(1, "X: %.2f Y: %.2f", odom.pose.x, odom.pose.y);

        pros::delay(20);
    }
}