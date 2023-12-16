#pragma once
#include "PID.hpp"
#include "main.h"
#include "position.hpp"

namespace odom {

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

void moveVelocity(double left, double right);
void move(double left, double right);

/**
 * Autonomous enum
 */
enum class Autonomous { ScoreLeft, ScoreSimple, TouchBar, Skills, None };

/**
 * follow pure pursuit path
 */
void follow(cstd::vector<odom::RobotPosition>& pathPoints, float lookahead,
            int timeout, bool forwards, bool async);

extern Autonomous autonomous;

}  // namespace odom