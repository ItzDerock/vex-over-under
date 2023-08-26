#include "./odom.hpp"
#include "../config.hpp"
#include "main.h"

// Task to update the odom
pros::Task *odomTask = nullptr;

struct {
  double left, right, center, theta;
} prevSensors = {0, 0, 0, 0};

struct {
  double left, right, theta;
} resetValues = {0, 0, 0};

odom::RobotPosition state = {0, 0, 0};

void odom::update() {
  // skip runs when all sensors are not initialized

  // 1. Store the current encoder values
  auto left = left_tracking_wheel->get_position();
  auto right = right_tracking_wheel->get_position();
  auto center = middle_tracking_wheel->get_position();

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
  auto newTheta =
      resetValues.theta + (left - right) / (left_tracking_wheel.getOffset() +
                                            right_tracking_wheel.getOffset());

  printf("newTheta: %f\n", newTheta);

  // 6. Calculate change in orientation
  auto dTheta = newTheta - state.theta;

  // 7. Calculate local offset for dTheta = 0
  odom::XY localOffset = {0, 0};

  if (dTheta == 0) {
    localOffset.x = dC;
    localOffset.y = dR;
  } else {
    // 8. Otherwise, calculate local offset with formula.
    localOffset.x = 2 * sin(dTheta / 2) *
                    (dC / dTheta + (middle_tracking_wheel.getOffset()));
    localOffset.y = 2 * sin(dTheta / 2) *
                    (dR / dTheta + (right_tracking_wheel.getOffset()));
  }

  // 9. Calculate the average orientation
  auto thetam = state.theta + dTheta / 2;

  // 10. Calculate the global offset
  // 10. Calculate global offset
  auto globalOffset = {0, 0};

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
}

void odom::init() {
  if (odomTask == nullptr) {
    odomTask = new pros::Task([]() {
      while (true) {
        update();
        pros::delay(10);
      }
    });
  }
}

void odom::reset(odom::RobotPosition startState) {
  // stop task
  if (odomTask != nullptr) {
    odomTask->remove();
    odomTask = nullptr;
  }

  // reset encoders
  left_tracking_wheel.reset();
  right_tracking_wheel.reset();
  middle_tracking_wheel.reset();

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
}

void odom::reset() {
  // default to 0, 0, 0
  reset({0, 0, 0});
}

odom::RobotPosition odom::getPosition(bool degrees) {
  if (!degrees)
    return state;

  return {state.x, state.y, state.theta * (180 / M_PI)};
}

odom::RobotPosition odom::getPosition() { return getPosition(false); }