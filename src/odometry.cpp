# include "main.h"
#include "pros/rotation.hpp"
#include "helpers.hpp"

//create a struct to store pose in (x,y,theta)
struct Pose {
    double x;
    double y;
    double theta; // in radians

    Pose(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {}
};

//create a class for my tracking wheels
class TrackingWheel {
public:
    pros::Rotation* encoder;
    float wheelDiameter;
    float gearRatio;
    float offset;
    float prevTravel = 0;

    TrackingWheel(pros::Rotation* enc, float diameter, float offset_, float gearRatio_=1)
        : encoder(enc), wheelDiameter(diameter), offset(offset_), gearRatio(gearRatio_) {}

    float getDelta() {
        float travel = (encoder->get_position() / 360.0) * (wheelDiameter * M_PI) * gearRatio;
        float delta = travel - prevTravel;
        prevTravel = travel;
        return delta;
    }
};

//create a class for my odometry
class Odometry {
public:
    TrackingWheel vertical;
    TrackingWheel horizontal;
    pros::Imu imu;

    Pose pose;
    double prevTheta = 0;
    bool newPose = true;

    Odometry(TrackingWheel v, TrackingWheel h, pros::Imu imu) : vertical(v), horizontal(h), imu(imu) {}

    void setPose(double x, double y, double thetaDeg) {
        pose = Pose(x, y, thetaDeg * M_PI / 180.0f);
        imu.set_rotation(thetaDeg);
        prevTheta = pose.theta;
        newPose = true;
        }
    
    void update() {
        double deltaVertical = vertical.getDelta();
        double deltaHorizontal = horizontal.getDelta();
        
        double thetaNow = imu.get_rotation() * M_PI / 180.0f;
        float deltaTheta = newPose ? 0 : angleError(thetaNow, prevTheta);
        newPose = false;

        double localX, localY;

        if(fabs(deltaTheta) < 1e-6) {
            localX = deltaHorizontal;
            localY = deltaVertical;
        } else {
            double radius = deltaVertical / deltaTheta;
            localX = 2 * sin(deltaTheta / 2.0) * (radius + horizontal.offset);
            localY = 2 * sin(deltaTheta / 2.0) * (radius + vertical.offset);
        }

        double avgTheta = prevTheta + deltaTheta / 2.0;
        double sinTheta = sin(avgTheta);
        double cosTheta = cos(avgTheta);

        pose.x += localY * sinTheta - localX * cosTheta;
        pose.y += localY * cosTheta + localX * sinTheta;
        pose.theta = thetaNow;

        prevTheta = thetaNow;
    }

private:
    double angleError(double a, double b) {
        double diff = a - b;
        while (diff > M_PI) diff -= 2*M_PI;
        while (diff < -M_PI) diff += 2*M_PI;
        return diff;
    }
};