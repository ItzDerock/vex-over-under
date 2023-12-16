#include "../config.hpp"
#include "robot/odom.hpp"

#define SETTLED_TIME 800  // milliseconds

odom::Autonomous odom::autonomous = odom::Autonomous::None;

std::shared_ptr<PIDController> odom::turnPID =
    std::make_shared<PIDController>(6, 0.01, 0.05);
std::shared_ptr<PIDController> odom::drivePID =
    std::make_shared<PIDController>(35, 0.01, 0.001);
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
    angularError = position.theta - targetPosition.theta;

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

    double left = sign * output + angularError;
    double right = sign * output - angularError;

    moveVelocity(left, right);

    // wait 10 milliseconds
    pros::delay(10);
  }

  // stop the motors
  moveVelocity(0, 0);
}

// utility functions for turning
/**
 * Calculates the error between two angles.
 * Expects angles in RADIANS!
 */
double angleError(double angle1, double angle2) {
  return std::remainder(angle1 - angle2, 2 * M_PI);
}

/**
 * Returns the angle in the range [0, 2PI]
 */
double angleSquish(double angle) { return fmod(angle, 2 * M_PI); }

void odom::turnTo(double theta) { odom::turnTo(theta, 5); }

void odom::turnTo(double theta, double timeout) {
  // "fix" the inputs
  timeout = timeout * 1000;
  theta = angleSquish(theta);

  unsigned int settledTime = 0;
  uint32_t start = pros::millis();

  std::cout << "theta: " << theta << std::endl;

  while (settledTime < SETTLED_TIME && (pros::millis() - start) < timeout) {
    RobotPosition position = getPosition();

    double error = angleError(theta, position.theta) * 180 / M_PI;

    std::cout << "angular error: " << error << std::endl;
    double output = turnPID->update(error);
    std::cout << "output: " << output << std::endl;

    if (error < 0.02) {
      settledTime += 10;
    }

    moveVelocity(-output, output);
    pros::delay(10);
  }

  moveVelocity(0, 0);
}