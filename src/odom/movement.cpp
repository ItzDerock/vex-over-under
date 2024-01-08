#include "../config.hpp"
#include "robot/odom.hpp"
#include "robot/utils.hpp"

#define SETTLED_TIME 3000  // milliseconds

odom::Autonomous odom::autonomous = odom::Autonomous::Skills;
// TODO: switch to NONE for comp

std::shared_ptr<PIDController> odom::turnPID =
    std::make_shared<PIDController>(5, 0, 25);
// std::make_shared<PIDController>(8, 0.08, 45);
std::shared_ptr<PIDController> odom::drivePID =
    std::make_shared<PIDController>(35, 0, 20);

std::shared_ptr<ExitCondition> odom::lateralLargeExit =
    std::make_shared<ExitCondition>(3, 500);
std::shared_ptr<ExitCondition> odom::lateralSmallExit =
    std::make_shared<ExitCondition>(1, 100);

std::shared_ptr<ExitCondition> odom::angularLargeExit =
    std::make_shared<ExitCondition>(3, 500);
std::shared_ptr<ExitCondition> odom::angularSmallExit =
    std::make_shared<ExitCondition>(1, 100);

/**
 * Returns the distance between two points
 */
double distance(odom::RobotPosition a, odom::RobotPosition b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void odom::moveVelocity(double left, double right) {
  drive_left_back->move_velocity(left);
  drive_left_front->move_velocity(left);
  drive_left_pto->move_velocity(left);
  drive_right_back->move_velocity(right);
  drive_right_front->move_velocity(right);
  drive_right_pto->move_velocity(right);
}

void odom::move(double left, double right) {
  drive_left_back->move(left);
  drive_left_front->move(left);
  drive_left_pto->move(left);
  drive_right_back->move(right);
  drive_right_front->move(right);
  drive_right_pto->move(right);
}

// void odom::moveDistance(double dist, double timeout) {
//   // find the target position's X and Y
//   RobotPosition initialPosition = getPosition();
//   double targetX = initialPosition.x + dist * sin(initialPosition.theta);
//   double targetY = initialPosition.y + dist * cos(initialPosition.theta);
//   RobotPosition targetPosition = {targetX, targetY, initialPosition.theta};

//   odom::moveTo(targetPosition.x, targetPosition.y, targetPosition.theta,
//                timeout, {}, false);
// }

void odom::moveDistance(double dist, double timeout) {
  int8_t sign = dist < 0 ? -1 : 1;

  // to finish movement, we should be settled for SETTLED_TIME
  unsigned int settledAmount = 0;
  double distanceError = infinity();
  double angularError = infinity();
  uint32_t startTime = pros::millis();
  timeout = timeout * 1000;

  // find the target position's X and Y
  RobotPosition initialPosition = getPosition();
  double targetX = initialPosition.x + dist * sin(initialPosition.theta);
  double targetY = initialPosition.y + dist * cos(initialPosition.theta);
  RobotPosition targetPosition = {targetX, targetY, initialPosition.theta};

  std::cout << "targetX: " << targetX << std::endl;
  std::cout << "targetY: " << targetY << std::endl;

  if (sign < 0) dist = dist * -1;

  // loop until we are settled
  while (settledAmount < SETTLED_TIME && pros::millis() - startTime < timeout) {
    // get the current position
    RobotPosition position = getPosition();

    // calculate the error
    // $$\text{Distance} = d_i - d_t$$
    distanceError = dist - distance(position, initialPosition);
    angularError = utils::radToDeg(position.theta - targetPosition.theta);

    std::cout << "distanceError: " << distanceError << std::endl;
    std::cout << "angularError: " << angularError << std::endl;

    // if we are settled, increment settledAmount
    if (fabs(distanceError) < 0.5) settledAmount += 10;
    // otherwise, reset settledAmount
    else
      settledAmount = 0;

    // calculate the output
    double output = drivePID->update(distanceError);
    double angularOutput = -1 * turnPID->update(angularError);

    std::cout << "output: " << output << std::endl;

    // set the motors
    // double left = output + angularOutput;
    // double right = output - angularOutput;

    double left = sign * output - angularError;
    double right = sign * output + angularError;

    moveVelocity(left, right);

    // wait 10 milliseconds
    pros::delay(10);
  }

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

  // reset PIDs and exit conditions
  drivePID->reset();
  lateralLargeExit->reset();
  lateralSmallExit->reset();
  turnPID->reset();
  angularLargeExit->reset();
  angularSmallExit->reset();

  // calculate target pose
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
  while (!timer.isUp() &&
         ((!lateralSettled ||
           (!angularLargeExit->getExit() && !angularSmallExit->getExit())) ||
          !close)) {
    // update position
    RobotPosition pose = getPosition(false, true);

    // convert from odometry angle to standard angle
    // current odom implementation uses a half-bearing and half-standard
    // calculation we must subtract from 2pi to get the standard angle
    // pose.theta = utils::angleSquish(2 * M_PI - pose.theta);

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

    printf("close: %d\n", close);
    printf("angularError: %f\n", utils::radToDeg(angularError));
    printf("robot theta: %f\n", utils::radToDeg(adjustedRobotTheta));
    if (close)
      printf("target theta: %f\n", utils::radToDeg(target.theta));
    else
      printf("carrot theta: %f\n",
             utils::radToDeg(utils::angleSquish(pose.angle(carrot))));

    printf("angle to carrot: %f\n",
           utils::radToDeg(utils::angleSquish(pose.angle(carrot))));
    printf("angle to target: %f\n", utils::radToDeg(pose.angle(target)));

    float lateralError = pose.distance(carrot);

    printf("distance: %f\n", lateralError);

    // only use cos when settling
    // otherwise just multiply by the sign of cos
    // maxSlipSpeed takes care of lateralOut
    if (close)
      lateralError *= cos(utils::angleError(pose.theta, pose.angle(carrot)));
    else
      lateralError *=
          utils::sgn(cos(utils::angleError(pose.theta, pose.angle(carrot))));

    printf("lateralError: %f\n", lateralError);

    // update exit conditions
    lateralSmallExit->update(lateralError);
    lateralLargeExit->update(lateralError);
    angularSmallExit->update(utils::radToDeg(angularError));
    angularLargeExit->update(utils::radToDeg(angularError));

    // get output from PIDs
    float lateralOut = drivePID->update(lateralError);
    float angularOut = turnPID->update(utils::radToDeg(angularError));

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
void odom::turnTo(double degrees, double timeout) {
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
    power = std::clamp(power, -127.0, 127.0);
    printf("power: %f\n", power);
    move(-power, power);

    pros::delay(10);
  }

  // stop the drivetrain
  move(0, 0);
}