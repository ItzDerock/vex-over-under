#include "../config.hpp"
#include "robot/odom.hpp"
#include "robot/utils.hpp"

int chainedMovementCount = 0;
odom::Autonomous odom::autonomous = odom::Autonomous::None;

std::shared_ptr<PIDController> odom::turnPID =
    std::make_shared<PIDController>(5, 0, 20);
std::shared_ptr<PIDController> odom::drivePID =
    std::make_shared<PIDController>(32, 0, 20);

std::shared_ptr<ExitCondition> odom::lateralLargeExit =
    std::make_shared<ExitCondition>(LATERAL_LARGE_EXIT);
std::shared_ptr<ExitCondition> odom::lateralSmallExit =
    std::make_shared<ExitCondition>(LATERAL_SMALL_EXIT);

std::shared_ptr<ExitCondition> odom::angularLargeExit =
    std::make_shared<ExitCondition>(ANGULAR_LARGE_EXIT);
std::shared_ptr<ExitCondition> odom::angularSmallExit =
    std::make_shared<ExitCondition>(ANGULAR_SMALL_EXIT);

/**
 * @brief Simple function to check if everything's settled
 */
inline bool isSettled() {
  return odom::lateralLargeExit->getExit() &&
         odom::angularLargeExit->getExit() &&  //
         odom::lateralSmallExit->getExit();
}

/**
 * @brief Updates the settlement requirements based on the current chained
 * movement amount
 */
inline void updateSettlement() {
  if (chainedMovementCount > 0) {
    odom::lateralLargeExit->setExit(CHAINED_LATERAL_LARGE_EXIT);
    odom::lateralSmallExit->setExit(CHAINED_LATERAL_SMALL_EXIT);
    odom::angularLargeExit->setExit(CHAINED_ANGULAR_LARGE_EXIT);
    odom::angularSmallExit->setExit(CHAINED_ANGULAR_SMALL_EXIT);
  } else {
    odom::lateralLargeExit->setExit(LATERAL_LARGE_EXIT);
    odom::lateralSmallExit->setExit(LATERAL_SMALL_EXIT);
    odom::angularLargeExit->setExit(ANGULAR_LARGE_EXIT);
    odom::angularSmallExit->setExit(ANGULAR_SMALL_EXIT);
  }
}

/**
 * @brief Moves the drivetrain (velocity control)
 */
void odom::moveVelocity(double left, double right) {
  drive_left_back->move_velocity(left);
  drive_left_front->move_velocity(left);
  drive_left_pto->move_velocity(left);
  drive_right_back->move_velocity(right);
  drive_right_front->move_velocity(right);
  drive_right_pto->move_velocity(right);
}

/**
 * @brief Moves the drivetrain (power control)
 */
void odom::move(double left, double right) {
  drive_left_back->move(left);
  drive_left_front->move(left);
  drive_left_pto->move(left);
  drive_right_back->move(right);
  drive_right_front->move(right);
  drive_right_pto->move(right);
}

void odom::moveDistance(double dist, double timeout, MoveToPoseParams params) {
  int8_t sign = dist < 0 ? -1 : 1;

  double distanceError = infinity();
  double angularError = infinity();

  // exit conditions
  utils::Timer timer(timeout);
  lateralSmallExit->reset();
  lateralLargeExit->reset();
  angularLargeExit->reset();
  angularSmallExit->reset();

  // reset the PIDs
  drivePID->reset();
  turnPID->reset();

  // find the target position's X and Y
  // note: RADIANS and STANDARD POSITION
  RobotPosition initialPosition = getPosition(false, true);
  double targetX = initialPosition.x + dist * cos(initialPosition.theta);
  double targetY = initialPosition.y + dist * sin(initialPosition.theta);
  RobotPosition targetPosition = {targetX, targetY, initialPosition.theta};

  // fix params
  params.forwards = sign > 0;

  odom::moveTo(targetX, targetY,
               utils::radToDeg(getPosition(false, false).theta), timeout,
               params, false);

  // stop the motors
  moveVelocity(0, 0);
}

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
void odom::moveTo(float x, float y, float theta, int timeout,
                  MoveToPoseParams params, bool async) {
  // if the function is async, run it in a new task
  if (async) {
    pros::Task task([&]() { moveTo(x, y, theta, timeout, params, false); });
    pros::delay(10);  // delay to give the task time to start
    return;
  }

  updateSettlement();
  if (chainedMovementCount > 0) {
    chainedMovementCount--;
  }

  // reset PIDs and exit conditions
  drivePID->reset();
  lateralLargeExit->reset();
  lateralSmallExit->reset();
  turnPID->reset();
  angularLargeExit->reset();
  angularSmallExit->reset();

  // create stall condition
  ExitCondition stallExit(params.stallThreshold, params.stallTime);
  RobotPosition startingPosition = getPosition(false, true);

  // calculate target pose
  // note: This variable is in RADIANS and in STANDARD POSITION.
  RobotPosition target(x, y, M_PI_2 - utils::degToRad(theta));
  if (!params.forwards)
    target.theta = fmod(target.theta + M_PI, 2 * M_PI);  // backwards movement

  // initialize vars used between iterations
  RobotPosition lastPose = getPosition();
  int distTravelled = 0;
  utils::Timer timer(timeout);
  bool close = false;
  bool lateralSettled = false;
  bool prevSameSide = false;
  float prevLateralOut = 0;  // previous lateral power
  float prevAngularOut = 0;  // previous angular power
  const int compState = pros::competition::get_status();

  // main loop
  while (!timer.isUp() && pros::competition::get_status() == compState &&
         ((!lateralSettled ||
           (!angularLargeExit->getExit() && !angularSmallExit->getExit())) ||
          !close)) {
    // note: This variable is in RADIANS and in STANDARD POSITION.
    RobotPosition pose = getPosition(false, true);

    // stall detection
    if (params.exitOnStall && distTravelled > 0) {
      // require to be at least 50% of the way to the target
      double distanceToTarget = pose.distance(target);
      double distanceTravelled = pose.distance(startingPosition);

      if (distanceTravelled > distanceToTarget) {
        if (stallExit.getExit()) {
          printf("stalled\n");
          break;
        }
        stallExit.update(getVelocity());
      }
    }

    // update distance travelled
    distTravelled += pose.distance(lastPose);
    lastPose = pose;

    // calculate distance to the target point
    const float distTarget = pose.distance(target);

    // check if the robot is close enough to the target to start settling
    if (distTarget < 7.5 && close == false) {
      close = true;
      params.maxSpeed = fmax(fabs(prevLateralOut), 60);
    }

    // check if the lateral controller has settled
    if (lateralLargeExit->getExit() && lateralSmallExit->getExit())
      lateralSettled = true;

    // calculate the carrot point
    RobotPosition carrot =
        target - RobotPosition(cos(target.theta), sin(target.theta), 0) *
                     params.lead * distTarget;
    printf("carrot: %f, %f\n", carrot.x, carrot.y);

    if (close) carrot = target;  // settling behavior

    // calculate if the robot is on the same side as the carrot point
    const bool robotSide =
        (pose.y - target.y) * -sin(target.theta) <=
        (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
    const bool carrotSide =
        (carrot.y - target.y) * -sin(target.theta) <=
        (carrot.x - target.x) * cos(target.theta) + params.earlyExitRange;
    const bool sameSide = robotSide == carrotSide;
    // exit if close
    if (!sameSide && prevSameSide && close && params.minSpeed != 0) break;
    prevSameSide = sameSide;

    // calculate error
    const float adjustedRobotTheta =
        params.forwards ? pose.theta : pose.theta + M_PI;

    const float angularError =
        close ? utils::angleError(adjustedRobotTheta, target.theta, true)
              : utils::angleError(adjustedRobotTheta,
                                  utils::angleSquish(pose.angle(carrot)), true);

    const float angularErrorDeg = utils::radToDeg(angularError);

    // DEBUG START
    // --------------------------------------------------------------------
    // printf("close: %d\n", close);
    // printf("angularError: %f\n", angularErrorDeg);
    // printf("robot theta: %f\n", utils::radToDeg(adjustedRobotTheta));
    // if (close)
    //   printf("target theta: %f\n", utils::radToDeg(target.theta));
    // else
    //   printf("carrot theta: %f\n",
    //          utils::radToDeg(utils::angleSquish(pose.angle(carrot))));

    // printf("angle to carrot: %f\n",
    //        utils::radToDeg(utils::angleSquish(pose.angle(carrot))));
    // printf("angle to target: %f\n", utils::radToDeg(pose.angle(target)));
    // DEBUG END
    // --------------------------------------------------------------------

    float lateralError = pose.distance(carrot);

    // printf("distance: %f\n", lateralError);

    // only use cos when settling
    // otherwise just multiply by the sign of cos
    // maxSlipSpeed takes care of lateralOut
    double cosTheta = cos(utils::angleError(pose.theta, pose.angle(carrot)));
    if (close)
      lateralError *= cosTheta;
    else
      lateralError *= utils::sgn(cosTheta);

    printf("lateralError: %f\n", lateralError);

    // update exit conditions
    lateralSmallExit->update(lateralError);
    lateralLargeExit->update(lateralError);
    angularSmallExit->update(angularErrorDeg);
    angularLargeExit->update(angularErrorDeg);

    // get output from PIDs
    float lateralOut = drivePID->update(lateralError);
    float angularOut = turnPID->update(angularErrorDeg);

    // apply restrictions on angular speed
    angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);

    // apply restrictions on lateral speed
    lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

    // constrain lateral output by max accel
    if (!close)
      lateralOut = utils::slew(lateralOut, prevLateralOut, params.slew);

    // constrain lateral output by the max speed it can travel at without
    // slipping
    const float radius = 1 / fabs(utils::getCurvature(pose, carrot));
    const float maxSlipSpeed(sqrt(params.chasePower * radius * 9.8));
    lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
    // prioritize angular movement over lateral movement
    const float overturn =
        fabs(angularOut) + fabs(lateralOut) - params.maxSpeed;
    if (overturn > 0) lateralOut -= lateralOut > 0 ? overturn : -overturn;

    // prevent moving in the wrong direction
    if (params.forwards && !close)
      lateralOut = std::fmax(lateralOut, 0);
    else if (!params.forwards && !close)
      lateralOut = std::fmin(lateralOut, 0);

    // constrain lateral output by the minimum speed
    if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0)
      lateralOut = fabs(params.minSpeed);
    if (!params.forwards && -lateralOut < fabs(params.minSpeed) &&
        lateralOut < 0)
      lateralOut = -fabs(params.minSpeed);

    // update previous output
    prevAngularOut = angularOut;
    prevLateralOut = lateralOut;

    // ratio the speeds to respect the max speed
    float leftPower = lateralOut + angularOut;
    float rightPower = lateralOut - angularOut;
    const float ratio =
        std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
    if (ratio > 1) {
      leftPower /= ratio;
      rightPower /= ratio;
    }

    // move the drivetrain
    move(leftPower, rightPower);

    // delay to save resources
    pros::delay(10);
  }

  // stop the drivetrain
  move(0, 0);
}

/**
 * @brief Turns to a given angle
 */
void odom::turnTo(double degrees, double timeout, double maxSpeed) {
  // reset angular controllers/exit conditions
  turnPID->reset();
  angularLargeExit->reset();
  angularSmallExit->reset();

  // ensure we are still in autonomous
  // and that we do not go past the timeout
  utils::Timer timer(timeout * 1000);
  uint8_t compState = pros::competition::get_status();

  while (!timer.isUp() && !angularLargeExit->getExit() &&
         !angularSmallExit->getExit() &&
         pros::competition::get_status() == compState) {
    // calculate error in degrees
    // this is because degrees makes it easier to tune the PID
    // as errors are larger
    RobotPosition pose = getPosition(true);
    double error = utils::angleError(pose.theta, degrees);

    printf("pose: %f\n", pose.theta);
    printf("target: %f\n", degrees);
    printf("error: %f\n", error);

    // calculate the output from the PID
    double power = turnPID->update(error);
    angularLargeExit->update(error);
    angularSmallExit->update(error);

    // constrain the output
    power = std::clamp(power, -maxSpeed, maxSpeed);
    printf("power: %f\n", power);
    move(-power, power);

    pros::delay(10);
  }

  // stop the drivetrain
  move(0, 0);
}

void odom::startChainedMovement(int amount) {
  chainedMovementCount = amount;
  updateSettlement();
}