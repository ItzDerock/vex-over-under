#pragma once
#include "main.h"
#include "pros/rotation.hpp"

// ODOMETRY
#define ODOMETRY_WHEEL_DIAMETER 2.75 // inches
#define ODOM_LEFT_PORT 8
#define ODOM_MIDDLE_PORT 9
#define ODOM_RIGHT_PORT 10

// DRIVETRAIN
#define DRIVE_LEFT_FRONT 1
#define DRIVE_LEFT_BACK 2
#define DRIVE_LEFT_PTO 3
#define DRIVE_RIGHT_FRONT 4
#define DRIVE_RIGHT_BACK 5
#define DRIVE_RIGHT_PTO 6

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

//// Drivetrain
extern SHARED(pros::Motor, drive_left_front);
extern SHARED(pros::Motor, drive_left_back);
extern SHARED(pros::Motor, drive_left_pto);
extern SHARED(pros::Motor, drive_right_front);
extern SHARED(pros::Motor, drive_right_back);
extern SHARED(pros::Motor, drive_right_pto);

extern pros::MotorGroup drive_left;
extern pros::MotorGroup drive_right;

#undef SHARED