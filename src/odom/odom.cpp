#include "./odom.hpp"

#include <math.h>

#include "../config.hpp"
#include "main.h"

/**
 * Odometry implementation is based on the following paper:
 * http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
 *
 * Movement functions are not based on that paper.
 */

std::shared_ptr<PIDController> odom::turnPID =
    std::make_shared<PIDController>(0.2, 0.01, 0.001);
std::shared_ptr<PIDController> odom::drivePID =
    std::make_shared<PIDController>(0.5, 0.01, 0.001);

// Task to update the odom
pros::Task *odomTask = nullptr;
pros::Mutex odom::mutex;

struct
{
  double left, right, center, theta;
} prevSensors = {0, 0, 0, 0};

struct
{
  double left, right, theta;
} resetValues = {0, 0, 0};

odom::RobotPosition state = {0, 0, 0};

/**
 * Since part of our odometry is based on internal sensors in the motors,
 * we need to normalize the data to account for different gear ratios.
 *
 * @param sensor The sensor to normalize
 */
double normalizeSensorData(OdomIntegratedSensor sensor)
{
  // return sensor.sensor->get_position() * (1/sensor.gear_ratio) * 600 / 360;
  return sensor.sensor->get_position() * sensor.gear_ratio * 4 * M_PI;
}

void odom::update()
{
  // lock mutex
  mutex.take();

  // 1. Store the current encoder values
  auto left = odom_left.sensor->get_position();
  auto right = odom_right.sensor->get_position();
  auto center = odom_middle.sensor->get_value() * 2.75 * M_PI;

  std::cout << "left: " << left << std::endl;

  // 2. Calculate delta values
  auto dL = left - prevSensors.left;
  auto dR = right - prevSensors.right;
  auto dC = center - prevSensors.center;

  // 3. Update the previous values
  prevSensors.left = left;
  prevSensors.right = right;
  prevSensors.center = center;

  // 4. total change since last reset
  // auto deltaLr = left - resetValues.left;
  // auto deltaRr = right - resetValues.right;

  // 5. Calculate new orientation
  auto newTheta = resetValues.theta +
                  (left - right) / (odom_left.offset + odom_right.offset);

  // 6. Calculate change in orientation
  auto dTheta = newTheta - state.theta;

  // 7. Calculate local offset for dTheta = 0
  RobotPosition localOffset = {0, 0, 0};

  if (dTheta == 0)
  {
    localOffset.x = dC;
    localOffset.y = dR;
  }
  else
  {
    // 8. Otherwise, calculate local offset with formula.
    localOffset.x = 2 * sin(dTheta / 2) * (dC / dTheta + (odom_middle.offset));
    localOffset.y = 2 * sin(dTheta / 2) * (dR / dTheta + (odom_right.offset));
  }

  // 9. Calculate the average orientation
  auto thetam = state.theta + dTheta / 2;

  // 10. Calculate the global offset
  RobotPosition globalOffset = {0, 0, 0};
  // int globalOffsetX = 0;
  // int globalOffsetY = 0;

  // convert local offset to polar coordinates
  double r =
      sqrt(localOffset.x * localOffset.x + localOffset.y * localOffset.y);
  double theta = atan2(localOffset.y, localOffset.x);

  // subtract thetam from the angle component
  theta -= thetam;

  // convert back to Cartesian coordinates
  globalOffset.x = r * cos(theta);
  globalOffset.y = r * sin(theta);

  // 11. Update the global position
  state.x += globalOffset.x;
  state.y += globalOffset.y;

  state.theta = newTheta;

  std::cout << "x: " << state.x << std::endl;
  std::cout << "y: " << state.y << std::endl;
  std::cout << "theta: " << state.theta << std::endl;

  // unlock mutex
  mutex.give();
}

void odom::init()
{
  if (odomTask != nullptr)
  {
    std::cout << "WARNING: odom::init() called when odomTask is not null"
              << std::endl;
    return;
  }

  odomTask = new pros::Task([]()
                            {
    while (true) {
      update();
      pros::delay(20);
    } });
}

void odom::reset(odom::RobotPosition startState)
{
  // aquire mutex
  mutex.take();

  // stop task
  if (odomTask != nullptr)
  {
    odomTask->remove();
    odomTask = nullptr;
  }

  // reset encoders
  // TODO: fix
  // odom_left.sensor->reset();
  // odom_right.sensor->reset();
  odom_middle.sensor->reset();

  // reset state
  // state = startState;
  state.x = startState.x;
  state.y = startState.y;
  state.theta = startState.theta;
  resetValues.theta = startState.theta;

  // reset prevSensors
  prevSensors = {0, 0, 0, 0};

  // restart task
  init();

  // release mutex
  mutex.give();
}

void odom::reset()
{
  // default to 0, 0, 0
  reset({0, 0, 0});
}

odom::RobotPosition odom::getPosition(bool degrees)
{
  // aquire mutex
  mutex.take();

  // get the state
  RobotPosition returnState =
      degrees ? RobotPosition(state.x, state.y, state.theta * (180 / M_PI))
              : state;

  // release mutex
  mutex.give();

  // return the state
  return returnState;
}

odom::RobotPosition odom::getPosition() { return getPosition(false); }

void odom::turnTo(double theta)
{
  // aquire mutex
  mutex.take();

  // get the current theta
  double currentTheta = state.theta;

  // release mutex
  mutex.give();

  // calculate the error
  double error = theta - currentTheta;

  // if error is greater than 180, subtract 360
  if (error > 180)
    error -= 360;
  // if error is less than -180, add 360
  else if (error < -180)
    error += 360;

  // while the error is greater than the allowed error
  while (fabs(error) > 0.5)
  {
    // aquire mutex
    mutex.take();

    // get the current theta
    currentTheta = state.theta;

    // release mutex
    mutex.give();

    // calculate the error
    error = theta - currentTheta;

    // if error is greater than 180, subtract 360
    if (error > 180)
      error -= 360;
    // if error is less than -180, add 360
    else if (error < -180)
      error += 360;

    // calculate the output
    double output = odom::turnPID->update(error);

    // set the motors
    drive_left_back->move(output);
    drive_left_front->move(output);
    drive_left_pto->move(output);
    drive_right_back->move(-output);
    drive_right_front->move(-output);
    drive_right_pto->move(-output);

    // wait
    pros::delay(20);
  }

  // stop the motors
  drive_left_back->move(0);
  drive_left_front->move(0);
  drive_left_pto->move(0);
  drive_right_back->move(0);
  drive_right_front->move(0);
  drive_right_pto->move(0);
}