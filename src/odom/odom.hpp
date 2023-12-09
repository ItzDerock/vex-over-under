#pragma once
#include "../algorithms/PID.hpp"
#include "main.h"

namespace odom {

struct RobotPosition {
  double x;
  double y;
  double theta;

  int getDegrees() { return (int)(theta * 180 / M_PI); }
  RobotPosition(double x, double y, double theta) : x(x), y(y), theta(theta) {}

  // subtract operator
  RobotPosition operator-(const RobotPosition& other) {
    return RobotPosition(x - other.x, y - other.y, theta - other.theta);
  }

  // add operator
  RobotPosition operator+(const RobotPosition& other) {
    return RobotPosition(x + other.x, y + other.y, theta + other.theta);
  }

  // multiply operator
  double operator*(const RobotPosition& other) {
    return this->x * other.x + this->y * other.y;
  }

  RobotPosition operator*(const double& other) {
    return RobotPosition(x * other, y * other, theta);
  }

  double distance(const RobotPosition& other) {
    return std::hypot(this->x - other.x, this->y - other.y);
  }

  odom::RobotPosition lerp(odom::RobotPosition other, double t) {
    return odom::RobotPosition(this->x + (other.x - this->x) * t,
                               this->y + (other.y - this->y) * t, this->theta);
  }
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
void updateLoop();

/**
 * Resets the odometry to a given position.
 */
void reset(RobotPosition startState);
void reset();

/**
 * Initializes the odometry task.
 */
void initalize();

/**
 * turn to a given angle (in degrees)
 */
void turnTo(double theta);
void turnTo(double theta, double timeout);

/**
 * Drive a certain distance (in inches)
 */
void moveDistance(double distance);
void moveDistance(double distance, double timeout);

/**
 * The odometry mutex. Use whenever you are reading values.
 */
extern pros::Mutex mutex;

void move(double left, double right);

/**
 * Autonomous enum
 */
enum class Autonomous { ScoreLeft, ScoreSimple, TouchBar, Skills, None };

/**
 * follow pure pursuit path
 */
void follow(const std::string& path, float lookahead, int timeout,
            bool forwards, bool async);

extern Autonomous autonomous;

}  // namespace odom