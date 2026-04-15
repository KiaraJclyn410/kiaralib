#include "odometry.hpp"

Pose::Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}

TrackingWheel::TrackingWheel(pros::Rotation* enc, float diameter, float offset_, float gearRatio_)
    : encoder(enc), wheelDiameter(diameter), gearRatio(gearRatio_), offset(offset_), prevTravel(0) {}

float TrackingWheel::getDelta() {
    float travel = (encoder->get_position() / 360.0) * (wheelDiameter * M_PI) * gearRatio;
    float delta = travel - prevTravel;
    prevTravel = travel;
    return delta;
}

Odometry::Odometry(TrackingWheel* v, TrackingWheel* h, pros::Imu* imu_)
    : vertical(v), horizontal(h), imu(imu_), pose(), prevTheta(0), newPose(true) {}

void Odometry::setPose(double x, double y, double thetaDeg) {
    pose = Pose(x, y, thetaDeg * M_PI / 180.0);
    imu->set_rotation(thetaDeg);
    prevTheta = pose.theta;
    newPose = true;
}

void Odometry::update() {
    double deltaVertical = vertical->getDelta();
    double deltaHorizontal = horizontal->getDelta();

    double thetaNow = imu->get_rotation() * M_PI / 180.0;
    double deltaTheta = newPose ? 0 : angleError(thetaNow, prevTheta);
    newPose = false;

    double localX, localY;

    if (fabs(deltaTheta) < 1e-6) {
        localX = deltaHorizontal;
        localY = deltaVertical;
    } else {
        double radius = deltaVertical / deltaTheta;
        localX = 2 * sin(deltaTheta / 2.0) * (radius + horizontal->offset);
        localY = 2 * sin(deltaTheta / 2.0) * (radius + vertical->offset);
    }

    double avgTheta = prevTheta + deltaTheta / 2.0;
    double sinTheta = sin(avgTheta);
    double cosTheta = cos(avgTheta);

    pose.x += localY * sinTheta - localX * cosTheta;
    pose.y += localY * cosTheta + localX * sinTheta;
    pose.theta = thetaNow;

    prevTheta = thetaNow;
}

double Odometry::angleError(double a, double b) {
    double diff = a - b;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}
