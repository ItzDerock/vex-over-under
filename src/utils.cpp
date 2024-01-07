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

/**
 * Converts degrees to radians
 */
double utils::degToRad(double deg) { return deg * M_PI / 180; }

/**
 * Converts radians to degrees
 */
double utils::radToDeg(double rad) { return rad * 180 / M_PI; }

/**
 * @brief Slew rate limiter
 *
 * @param target target value
 * @param current current value
 * @param maxChange maximum change. No maximum if set to 0
 * @return float - the limited value
 */
float utils::slew(float target, float current, float maxChange) {
  float change = target - current;
  if (maxChange == 0) return target;
  if (change > maxChange)
    change = maxChange;
  else if (change < -maxChange)
    change = -maxChange;
  return current + change;
}

/**
 * @brief Get the signed curvature of a circle that intersects the first pose
 * and the second pose
 *
 * @note The circle will be tangent to the theta value of the first pose
 * @note The curvature is signed. Positive curvature means the circle is going
 * clockwise, negative means counter-clockwise
 * @note Theta has to be in radians and in standard form. That means 0 is right
 * and increases counter-clockwise
 *
 * @param pose the first pose
 * @param other the second pose
 * @return float curvature
 */
float utils::getCurvature(odom::RobotPosition pose, odom::RobotPosition other) {
  // calculate whether the pose is on the left or right side of the circle
  float side = utils::sgn(std::sin(pose.theta) * (other.x - pose.x) -
                          std::cos(pose.theta) * (other.y - pose.y));
  // calculate center point and radius
  float a = -std::tan(pose.theta);
  float c = std::tan(pose.theta) * pose.x - pose.y;
  float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
  float d = std::hypot(other.x - pose.x, other.y - pose.y);

  // return curvature
  return side * ((2 * x) / (d * d));
}