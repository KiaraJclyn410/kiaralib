#pragma once
#include "main.h"
#include "pros/rotation.hpp"
#include "helpers.hpp"

struct Pose {
    double x;
    double y;
    double theta;

    Pose(double x = 0, double y = 0, double theta = 0);
};

class TrackingWheel {
public:
    pros::Rotation* encoder;
    float wheelDiameter;
    float gearRatio;
    float offset;
    float prevTravel;

    TrackingWheel(pros::Rotation* enc, float diameter, float offset_, float gearRatio_ = 1);

    float getDelta();
};

class Odometry {
public:
    TrackingWheel* vertical;
    TrackingWheel* horizontal;
    pros::Imu* imu;

    Pose pose;
    double prevTheta;
    bool newPose;

    Odometry(TrackingWheel* v, TrackingWheel* h, pros::Imu* imu);

    void setPose(double x, double y, double thetaDeg);
    void update();

private:
    double angleError(double a, double b);
};