#include "../config.hpp"
#include "odom.hpp"

#define SETTLED_TIME 1000  // milliseconds

void odom::moveDistance(double distance) {
  // to finish movement, we should be settled for SETTLED_TIME
  unsigned int settledAmount = 0;
  double distanceError = infinity();
  double angularError = infinity();

  // find the target position's X and Y
  RobotPosition position = getPosition();
  double targetX = position.x + distance * cos(position.theta);
  double targetY = position.y + distance * sin(position.theta);
  RobotPosition targetPosition = {targetX, targetY, position.theta};

  // loop until we are settled
  while (settledAmount < SETTLED_TIME) {
    // get the current position
    RobotPosition position = getPosition();

    // calculate the error
    distanceError =
        sqrt(pow(targetX - position.x, 2) + pow(targetY - position.y, 2));
    angularError = position.theta - targetPosition.theta;

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

    drive_left_back->move(left);
    drive_left_front->move(left);
    drive_left_pto->move(left);
    drive_right_back->move(right);
    drive_right_front->move(right);
    drive_right_pto->move(right);

    // wait 10 milliseconds
    pros::delay(10);
  }

  // stop the motors
  drive_left_back->move(0);
  drive_left_front->move(0);
  drive_left_pto->move(0);
  drive_right_back->move(0);
  drive_right_front->move(0);
  drive_right_pto->move(0);
}