#include "../config.hpp"
#include "odom.hpp"

#define SETTLED_TIME 1000  // milliseconds

/**
 * Returns the distance between two points
 */
double distance(odom::RobotPosition a, odom::RobotPosition b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void odom::moveDistance(double dist) {
  bool isBackwards = dist < 0;

  // to finish movement, we should be settled for SETTLED_TIME
  unsigned int settledAmount = 0;
  double distanceError = infinity();
  double angularError = infinity();

  // find the target position's X and Y
  RobotPosition initialPosition = getPosition();
  double targetX = initialPosition.x + dist * sin(initialPosition.theta);
  double targetY = initialPosition.y + dist * cos(initialPosition.theta);
  RobotPosition targetPosition = {targetX, targetY, initialPosition.theta};

  std::cout << "targetX: " << targetX << std::endl;
  std::cout << "targetY: " << targetY << std::endl;

  if (isBackwards) dist = dist * -1;

  // loop until we are settled
  while (settledAmount < SETTLED_TIME) {
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
    double angularOutput = turnPID->update(angularError);

    // set the motors
    double left = output + angularOutput;
    double right = output - angularOutput;

    if (isBackwards) {
      left = -1 * left;
      right = -1 * right;
    }

    drive_left_back->move_velocity(left);
    drive_left_front->move_velocity(left);
    drive_left_pto->move_velocity(left);
    drive_right_back->move_velocity(right);
    drive_right_front->move_velocity(right);
    drive_right_pto->move_velocity(right);

    // wait 10 milliseconds
    pros::delay(10);
  }

  // stop the motors
  drive_left_back->move_velocity(0);
  drive_left_front->move_velocity(0);
  drive_left_pto->move_velocity(0);
  drive_right_back->move_velocity(0);
  drive_right_front->move_velocity(0);
  drive_right_pto->move_velocity(0);
}