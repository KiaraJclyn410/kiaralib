#include "helpers.hpp"

double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

double radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}

double normalizeDegrees(double deg) {
    while (deg > 180) deg -= 360;
    while (deg < -180) deg += 360;
    return deg;
}

double normalizeRadians(double rad) {
    while (rad > M_PI) rad -= 2 * M_PI;
    while (rad < -M_PI) rad += 2 * M_PI;
    return rad;
}

double getTurnError(double target, double current) {
    return normalizeDegrees(target - current);
}

double getDistance(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

double getAngleToPoint(double x1, double y1, double x2, double y2) {
    return std::atan2(y2 - y1, x2 - x1);
}
