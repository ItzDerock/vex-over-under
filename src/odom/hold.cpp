/**
 * Holds code related to holding the robot in place
 */

#include <atomic>

#include "main.h"
#include "robot/odom.hpp"
#include "robot/utils.hpp"

/****** CONSTANTS ******/
void hold_task_fn(void *param);  // forward declaration
pros::Task *hold_task = new pros::Task(
    hold_task_fn, NULL, TASK_PRIORITY_DEFAULT, 0x500, "Hold Task");
std::atomic<float> targetAngle(-1);

// it is recommended to keep the I term at 0
// since integral windup can be a problem
std::shared_ptr<PIDController> holdPID =
    std::make_shared<PIDController>(5, 0, 20);
/***********************/

void hold_task_fn(void *param) {
  holdPID->reset();
  while (true) {
    // -1 = do not hold
    if (targetAngle == -1) {
      pros::delay(20);
      continue;
    }

    // Calculate error
    odom::RobotPosition pose = odom::getPosition(true);  // degrees
    double error = utils::angleError(pose.theta, targetAngle);

    // Calculate output
    double output = holdPID->update(error);
    odom::move(-output, output);

    pros::delay(20);
  }
}

void odom::holdAngle(double angle) { targetAngle = angle; }