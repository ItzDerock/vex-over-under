#pragma once
#include "../algorithms/PID.hpp"
#include "main.h"

namespace odom
{

  struct RobotPosition
  {
    double x;
    double y;
    double theta;

    int getDegrees() { return (int)(theta * 180 / M_PI); }
    RobotPosition(double x, double y, double theta) : x(x), y(y), theta(theta) {}
  };

  /**
   * PID controller for odom-related tasks.
   */
  extern std::shared_ptr<PIDController> turnPID;
  extern std::shared_ptr<PIDController> drivePID;

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

  /**
   * turn to a given angle
   */
  void turnTo(double theta);

  /**
   * The odometry mutex. Use whenever you are reading values.
   */
  extern pros::Mutex mutex;

} // namespace odom