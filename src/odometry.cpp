#include "odometry.hpp"
#include <cmath>

Pose::Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}

TrackingWheel::TrackingWheel(pros::Rotation* enc, double diameter, double offset_, double gearRatio_)
    : encoder(enc), wheelDiameter(diameter), gearRatio(gearRatio_), offset(offset_), prevTravel(0) {}

double TrackingWheel::getTravel() const {
    // 36,000 centidegrees = 1 revolution
    return (encoder->get_position() / 36000.0) * (wheelDiameter * M_PI) * gearRatio; 
}

double TrackingWheel::getDelta() {
    double travel = getTravel();
    double delta = travel - prevTravel;
    prevTravel = travel;
    return delta;
}

void TrackingWheel::reset() {
    prevTravel = getTravel();
}

Odometry::Odometry(TrackingWheel* v, TrackingWheel* h, pros::Imu* imu_)
    : vertical(v), horizontal(h), imu(imu_), pose(), prevTheta(0), newPose(true) {}

void Odometry::setPose(double x, double y, double thetaDeg) {
    pose = Pose(x, y, thetaDeg * M_PI / 180.0);
    imu->set_rotation(thetaDeg);
    vertical->reset();
    horizontal->reset();
    prevTheta = pose.theta;
    newPose = true;
}

void Odometry::update() {
    // 1. Get encoder deltas
    double deltaVertical = -(vertical->getDelta());
    double deltaHorizontal = horizontal->getDelta();

    // 2. Calculate change in heading using your angleError function
    // Ensure theta is in Radians for the math below!
    double thetaNow = imu->get_rotation() * M_PI / 180.0; 

    if (std::isnan(thetaNow) || std::isinf(thetaNow)) {
    return; // Skip this frame if the sensor glitched
}
    double deltaTheta = newPose ? 0 : angleError(thetaNow, prevTheta);
    newPose = false;

    double localX = 0, localY = 0;

    if (fabs(deltaTheta) < 0.0005) { // Increased threshold for safety
        localX = deltaHorizontal;
        localY = deltaVertical;
    } else {
        double common = 2.0 * sin(deltaTheta / 2.0) / deltaTheta;
        localX = common * (deltaHorizontal - (deltaTheta * horizontal->offset));
        localY = common * (deltaVertical - (deltaTheta * vertical->offset));
    }

    // CRITICAL SAFETY CHECK: 
    // If for ANY reason localX or localY is NaN, stop here and don't ruin the pose!
    if (std::isnan(localX) || std::isnan(localY) || std::isinf(localX) || std::isinf(localY)) {
        return; 
    }

    // 4. Global transformation
    // avgTheta represents the orientation of the robot halfway through the movement
    double avgTheta = prevTheta + (deltaTheta / 2.0);
    
    double s = sin(avgTheta);
    double c = cos(avgTheta);

    // Apply rotation matrix to convert local movement to global field coordinates
    pose.x += localY * s + localX * c;
    pose.y += localY * c - localX * s;

    // 5. Update state for next loop
    pose.theta = thetaNow;
    prevTheta = thetaNow;
}

double Odometry::angleError(double a, double b) {
    double diff = a - b;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}
