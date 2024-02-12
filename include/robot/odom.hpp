#pragma once
#include "ExitCondition.hpp"
#include "PID.hpp"
#include "main.h"
#include "position.hpp"

namespace odom {

/**
 * PID controller for odom-related tasks.
 */
extern std::shared_ptr<PIDController> turnPID;
extern std::shared_ptr<PIDController> drivePID;
extern std::shared_ptr<ExitCondition> lateralLargeExit;
extern std::shared_ptr<ExitCondition> lateralSmallExit;
extern std::shared_ptr<ExitCondition> angularLargeExit;
extern std::shared_ptr<ExitCondition> angularSmallExit;

/**
 * Returns the current robot location, by default in radians.
 * @param degrees whether to return the angle in degrees
 * @param standardPos whether to return the angle in standard form (0 deg is +x)
 */
RobotPosition getPosition(bool degrees = false, bool standardPos = false);

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
void turnTo(double degrees, double timeout = 5, double maxSpeed = 127.0);

/**
 * Drive a certain distance (in inches)
 */
void moveDistance(double distance, double timeout = 10'000, float slew = 3.5);

struct MoveToPoseParams {
  float maxSpeed = 127;
  float minSpeed = 0;
  float chasePower = 10;
  float lead = 0;
  float earlyExitRange = 0;
  float slew = 3.5;
  bool forwards = true;
};

/**
 * @brief Move the chassis towards the target pose
 *
 * Uses the boomerang controller
 *
 * @param x x location
 * @param y y location
 * @param theta target heading in degrees.
 * @param timeout longest time the robot can spend moving
 *
 * @param maxSpeed the maximum speed the robot can move at. 127 at default
 * @param async whether the function should be run asynchronously. true by
 * default
 */
void moveTo(float x, float y, float theta, int timeout, MoveToPoseParams params,
            bool async);
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
void follow(std::shared_ptr<std::vector<odom::RobotPosition>> pathPoints,
            float lookahead, int timeout, bool forwards, bool async);

/**
 * Loads the given paths into the cache.
 */
void loadPaths(std::vector<std::string> const& files);

/**
 * Returns a path from the cache.
 */
std::shared_ptr<std::vector<odom::RobotPosition>> getPath(
    std::string const& path);

extern Autonomous autonomous;

}  // namespace odom