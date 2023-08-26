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

// odometry sensors
extern SHARED(pros::Rotation, left_tracking_wheel);
extern SHARED(pros::Rotation, middle_tracking_wheel);
extern SHARED(pros::Rotation, right_tracking_wheel);

#undef SHARED