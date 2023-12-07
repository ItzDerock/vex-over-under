#include "robot/utils.hpp"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * Calculates the error between two angles.
 * Expects angles in RADIANS!
 */
double utils::angleError(double angle1, double angle2) {
  return std::remainder(angle1 - angle2, 2 * M_PI);
}

/**
 * Returns the angle in the range [0, 2PI]
 */
double utils::angleSquish(double angle) { return fmod(angle, 2 * M_PI); }
