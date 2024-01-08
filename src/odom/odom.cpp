#include "robot/odom.hpp"

#include <math.h>

#include "../config.hpp"
#include "main.h"
#include "robot/logger.hpp"
#include "robot/utils.hpp"

#define ODOM_DEBUG false

/**
 * Odometry implementation is based on the following paper:
 * http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
 *
 * Movement functions are not based on that paper.
 */

// Task to update the odom
pros::Task* odomTask = nullptr;
pros::Mutex odom::mutex;

struct {
  double left, right, center, theta;
} prevSensors = {0, 0, 0, 0};

struct {
  double left, right, theta;
} resetValues = {0, 0, 0};

odom::RobotPosition state = {0, 0, 0};

/**
 * Since part of our odometry is based on internal sensors in the motors,
 * we need to normalize the data to account for different gear ratios.
 *
 * @note - marked inline to reduce overhead of function calls
 *       - marked static so only accessible in odom.cpp
 *
 * @param sensor The sensor to normalize
 */
inline static double normalizeSensorData(double position,
                                         BasicOdomSensor sensor) {
  return (position * sensor.gear_ratio) / 360 * sensor.wheel_size * M_PI;
}

double readDrivetrainSensor(
    std::vector<std::shared_ptr<pros::Motor>> const& motors) {
  // find the median value
  // quick hack since we only have 3 motors
  double values[3] = {motors[0]->get_position(), motors[1]->get_position(),
                      motors[2]->get_position()};

  if (values[0] == values[1] || values[0] == values[2]) return values[0];
  if (values[1] == values[2]) return values[1];

  // as fallback, calculate the average
  // std::cerr << "Large discrepancy between motor values!" << std::endl
  //           << "Values: " << values[0] << ", " << values[1] << ", " <<
  //           values[2]
  //           << std::endl;
  return (values[0] + values[1] + values[2]) / 3;
}

void odom::update() {
  // lock mutex
  mutex.take();

  // 1. Store the current encoder values
  double left = readDrivetrainSensor(drive_left);
  double right = readDrivetrainSensor(drive_right);
  double center = (double)odom_middle.sensor->get_value();

  printf("left: %f, right: %f, center: %f\n", left, right, center);

  // 2. Calculate delta values
  //  (2.1) Conver tto distance of wheel travel (inches)
  double dL = normalizeSensorData(left - prevSensors.left, odom_left);
  double dR = normalizeSensorData(right - prevSensors.right, odom_right);
  double dC = normalizeSensorData(center - prevSensors.center, odom_middle);

  // 3. Update the previous values
  prevSensors.left = left;
  prevSensors.right = right;
  prevSensors.center = center;

  // 4. total change since last reset
  // auto deltaLr = left - resetValues.left;
  // auto deltaRr = right - resetValues.right;

  // 5. Calculate new orientation
  // double newTheta = resetValues.theta + inertial->get_heading() * M_PI / 180;
  // if (newTheta > 2 * M_PI) newTheta -= 2 * M_PI;
  // newTheta = utils::angleSquish(newTheta);
  auto newTheta =
      resetValues.theta + (normalizeSensorData(left, odom_left) -
                           normalizeSensorData(right, odom_right)) /
                              (odom_left.offset + odom_right.offset);

  // 6. Calculate change in orientation
  double dTheta = newTheta - state.theta;

  // 7. Calculate local offset for dTheta = 0
  RobotPosition localOffset = {0, 0, 0};

  if (dTheta == 0) {
    localOffset.x = dC;
    localOffset.y = dR;
  } else {
    // 8. Otherwise, calculate local offset with formula.
    localOffset.x = 2 * sin(dTheta / 2) * (dC / dTheta + (odom_middle.offset));
    localOffset.y = 2 * sin(dTheta / 2) * (dR / dTheta + (odom_right.offset));
  }

  // 9. Calculate the average orientation
  double thetam = state.theta + dTheta / 2;

  // state.x += localOffset.y * sin(thetam);
  // state.y += localOffset.y * cos(thetam);
  // state.x += localOffset.x * -cos(thetam);
  // state.y += localOffset.x * sin(thetam);
  // state.theta = newTheta;

  // 10. Calculate the global offset
  RobotPosition globalOffset = {0, 0, 0};

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

#if ODOM_DEBUG
  logger::log(logger::Route::RobotPosition, {state.x, state.y, state.theta});

  // I wish there was a more elegant way to do this
  logger::log(logger::Route::RobotVelocity,
              {
                  drive_left_back->get_actual_velocity(),
                  (double)drive_left_back->get_target_velocity(),
                  drive_right_back->get_actual_velocity(),
                  (double)drive_right_back->get_target_velocity(),
              });
#endif

  // unlock mutex
  mutex.give();
}

void odom::updateLoop() {
  while (true) {
    update();
    pros::delay(10);
  }
}

void odom::initalize() {
  if (odomTask != nullptr) {
    std::cout << "WARNING: odom::init() called when odomTask is not null"
              << std::endl;
    return;
  }

  odomTask = new pros::Task(updateLoop);
}

// macro to handle errors properly
#define CHECK_SUCCESS(fn, name)                           \
  if (fn != 1) {                                          \
    std::cerr << "FAILED TO RESET ODOMETRY!" << std::endl \
              << "errorno: " << errno << std::endl        \
              << "at: " << name << std::endl;             \
  }

void odom::reset(odom::RobotPosition startState) {
  // aquire mutex
  mutex.take();

  // stop task
  bool taskRunning = odomTask != nullptr;
  if (taskRunning) {
    odomTask->remove();
    odomTask = nullptr;
  }

  // reset encoders
  for (auto motor : drive_left) {
    CHECK_SUCCESS(motor->set_zero_position(0), "drive_left");
  }

  for (auto motor : drive_right) {
    CHECK_SUCCESS(motor->set_zero_position(0), "drive_right");
  }

  CHECK_SUCCESS(odom_middle.sensor->reset(), "odom_middle");
  CHECK_SUCCESS(inertial->reset(true), "odom_imu");

  // reset state
  // state = startState;
  state.x = startState.x;
  state.y = startState.y;
  state.theta = startState.theta;
  resetValues.theta = startState.theta;

  // reset prevSensors
  prevSensors = {0, 0, 0, 0};

  // delay 10ms to let the sensors reset
  pros::delay(10);

  // restart task
  if (taskRunning) initalize();

  // release mutex
  mutex.give();
}

void odom::reset() { reset({0, 0, 0}); }

odom::RobotPosition odom::getPosition(bool degrees, bool standardPos) {
  // aquire mutex
  mutex.take();

  // get the state
  RobotPosition returnState =
      degrees ? RobotPosition(state.x, state.y, state.theta * (180 / M_PI))
              : state;

  // bearing -> standard form
  if (standardPos) {
    returnState.theta = utils::angleSquish(M_PI_2 - returnState.theta);
  }

  // release mutex
  mutex.give();

  // return the state
  return returnState;
}