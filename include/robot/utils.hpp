#pragma once

namespace utils {

/**
 * Calculates the error between two angles.
 * Expects angles in RADIANS!
 */
double angleError(double angle1, double angle2);

/**
 * Returns the angle in the range [0, 2PI]
 */
double angleSquish(double angle);

}  // namespace utils