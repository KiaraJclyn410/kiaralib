#pragma once
#include "pros/rotation.hpp"
#include "pros/imu.hpp"

struct Pose {
    double x;
    double y;
    double theta;

    Pose(double x = 0, double y = 0, double theta = 0);
};

class TrackingWheel {
public:
    pros::Rotation* encoder;
    double wheelDiameter;
    double gearRatio;
    double offset;
    double prevTravel;

    TrackingWheel(pros::Rotation* enc, double diameter, double offset_, double gearRatio_ = 1);

    double getTravel() const;
    double getDelta();
    void reset();
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