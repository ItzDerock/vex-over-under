#pragma once

#include "main.h"
#include "robot/odom.hpp"
namespace chassis {

pros::Mutex movementMutex();

struct PurePursuitPoint {
  odom::RobotPosition position;
  float velocity;
};

}