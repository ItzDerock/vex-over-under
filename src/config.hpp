#pragma once
#include "main.h"
#include "pros/rotation.hpp"

// ODOMETRY
#define ODOMETRY_WHEEL_DIAMETER 2.75 // inches
#define ODOM_LEFT_PORT 8
#define ODOM_MIDDLE_PORT 9
#define ODOM_RIGHT_PORT 10

/*************************
 * VARIABLE DECLARATIONS *
 *************************/

// useful macro for quickly defining a shared_ptr
#define SHARED(type, name) std::shared_ptr<type> name

//// Odometry

struct OdomSensor {
  SHARED(pros::Rotation, sensor);
  double offset;
};

// odometry sensors
// no need to use shared pointers
extern OdomSensor odom_left;
extern OdomSensor odom_right;
extern OdomSensor odom_middle;

#undef SHARED