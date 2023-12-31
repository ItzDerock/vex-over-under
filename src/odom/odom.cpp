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
pros::Task *odomTask = nullptr;
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

void odom::update() {
  // lock mutex
  mutex.take();

  // 1. Store the current encoder values
  double left = odom_left.sensor->get_position();
  double right = odom_right.sensor->get_position();
  double center = (double)odom_middle.sensor->get_value();

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
  double newTheta = resetValues.theta + inertial->get_heading() * M_PI / 180;
  if (newTheta > 2 * M_PI) newTheta -= 2 * M_PI;
  // newTheta = utils::angleSquish(newTheta);
  // auto newTheta = resetValues.theta +
  //                 (left - right) / (odom_left.offset + odom_right.offset);

  // 6. Calculate change in orientation
  double dTheta = newTheta - state.theta;

  // 7. Calculate local offset for dTheta = 0
  RobotPosition localOffset = {0, 0, 0};

  if (dTheta == 0) {
    localOffset.x = dR;
    localOffset.y = dC;
  } else {
    // 8. Otherwise, calculate local offset with formula.
    localOffset.x = 2 * sin(dTheta / 2) * (dR / dTheta + (odom_right.offset));
    localOffset.y = 2 * sin(dTheta / 2) * (dC / dTheta + (odom_middle.offset));
  }

  // 9. Calculate the average orientation
  double thetam = state.theta + dTheta / 2;

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
  CHECK_SUCCESS(odom_left.sensor->set_zero_position(0), "odom_left");
  CHECK_SUCCESS(odom_right.sensor->set_zero_position(0), "odom_right");
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

  // restart task
  if (taskRunning) initalize();

  // release mutex
  mutex.give();
}

void odom::reset() {
  // default to 0, 0, 90deg
  reset({0, 0, M_PI / 2});
}

odom::RobotPosition odom::getPosition(bool degrees) {
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