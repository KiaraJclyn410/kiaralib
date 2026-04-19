#pragma once
#include "helpers.hpp"
#include <cmath>

// Constants
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Unit Conversions
double degToRad(double deg);
double radToDeg(double rad);

// Angle Normalization
double normalizeDegrees(double deg);
double normalizeRadians(double rad);
double getTurnError(double target, double current);

// 2D Math
double getDistance(double x1, double y1, double x2, double y2);
double getAngleToPoint(double x1, double y1, double x2, double y2);

double getTurnError(double target, double current);