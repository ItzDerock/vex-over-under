#include "../config.hpp"
#include "odom.hpp"

#define SETTLED_TIME 1500  // milliseconds

/**
 * Returns the distance between two points
 */
double distance(odom::RobotPosition a, odom::RobotPosition b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void move(double left, double right) {
  drive_left_back->move_velocity(left);
  drive_left_front->move_velocity(left);
  drive_left_pto->move_velocity(left);
  drive_right_back->move_velocity(right);
  drive_right_front->move_velocity(right);
  drive_right_pto->move_velocity(right);
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
    double angularOutput = turnPID->update(angularError);

    // set the motors
    // double left = output + angularOutput;
    // double right = output - angularOutput;

    double left = sign * output + angularError;
    double right = sign * output - angularError;

    move(left, right);

    // wait 10 milliseconds
    pros::delay(10);
  }

  // stop the motors
  move(0, 0);
}

void odom::turnTo(double theta) { odom::turnTo(theta, 5); }

void odom::turnTo(double theta, double timeout) {
  unsigned int settledTime = 0;
  uint32_t start = pros::millis();
  timeout = timeout * 1000;

  // fix theta if <0 >2M_PI
  if (theta < 0) theta += 2 * M_PI;
  if (theta > 2 * M_PI) theta -= 2 * M_PI;

  while (settledTime < SETTLED_TIME && (pros::millis() - start) < timeout) {
    RobotPosition position = getPosition();

    // double error = position.theta > theta ? position.theta - theta
    //                                       : theta - position.theta;

    double error = theta - position.theta;

    std::cout << "angular error: " << error << std::endl;
    double output = turnPID->update(error);

    if (error < 0.02) {
      settledTime += 10;
    }

    move(output, -output);
    pros::delay(10);
  }

  move(0, 0);
}

// void odom::turnTo(double theta) {
//   // get the current theta
//   double currentTheta = getPosition().theta;

//   // calculate the error
//   double error = theta - currentTheta;

//   // if error is greater than 180, subtract 360
//   if (error > M_PI) error -= 2 * M_PI;
//   // if error is less than -180, add 360
//   else if (error < -M_PI)
//     error += 2 * M_PI;

//   // while the error is greater than the allowed error
//   while (fabs(error) > 0.02) {
//     currentTheta = getPosition().theta;

//     // calculate the error
//     error = theta - currentTheta;

//     std::cout << "error: " << error << std::endl;

//     // if error is greater than 180, subtract 360
//     if (error > M_PI) error -= 2 * M_PI;
//     // if error is less than -180, add 360
//     else if (error < -M_PI)
//       error += 2 * M_PI;

//     // calculate the output
//     double output = odom::turnPID->update(error);

//     // set the motors
//     drive_left_back->move_velocity(output);
//     drive_left_front->move_velocity(output);
//     drive_left_pto->move_velocity(output);
//     drive_right_back->move_velocity(-output);
//     drive_right_front->move_velocity(-output);
//     drive_right_pto->move_velocity(-output);

//     // wait
//     pros::delay(20);
//   }

//   // stop the motors
//   drive_left_back->move(0);
//   drive_left_front->move(0);
//   drive_left_pto->move(0);
//   drive_right_back->move(0);
//   drive_right_front->move(0);
//   drive_right_pto->move(0);
// }