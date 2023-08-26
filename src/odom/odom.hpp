#pragma once
#include "main.h"

namespace odom {

struct RobotPosition {
  double x;
  double y;
  double theta;

  int getDegrees() { return (int)(theta * 180 / M_PI); }
  RobotPosition(double x, double y, double theta) : x(x), y(y), theta(theta) {}
};

/**
 * Returns the current robot location, by default in radians.
 */
RobotPosition getPosition(bool degrees);
RobotPosition getPosition();

/**
 * Updates the odoemtry position
 */
void update();

/**
 * Resets the odometry to a given position.
 */
void reset(RobotPosition startState);
void reset();

/**
 * Initializes the odometry task.
 */
void init();

} // namespace odom