#pragma once
#include "main.h"
#include "pros/adi.hpp"

// ODOMETRY
#define ODOMETRY_TICKS_PER_INCH 360.0  // ticks per inch
#define ODOMETRY_WHEEL_DIAMETER 2.75   // inches
#define ODOM_MIDDLE_PORT 'b', 'c'
#define ODOM_INERTIAL 16

// DRIVETRAIN
#define DRIVETRAIN_GEAR_RATIO /* input 32 -> output 72 */ 0.5
#define DRIVE_LEFT_FRONT 1
#define DRIVE_LEFT_BACK 2
#define DRIVE_LEFT_PTO 3
#define DRIVE_RIGHT_FRONT 4
#define DRIVE_RIGHT_BACK 5
#define DRIVE_RIGHT_PTO 6

// CATAPULT
#define CATAPULT_PORT 15
#define CATAPULT_ROT_PORT 10

// WINGS
#define WINGS_PORT 'a'

// BLOCKER
#define BLOCKER_PORT 'e'

/*************************
 * VARIABLE DECLARATIONS *
 *************************/

// useful macro for quickly defining a shared_ptr
#define SHARED(type, name) std::shared_ptr<type> name

//// Odometry

struct OdomSensor {
  SHARED(pros::adi::Encoder, sensor);
  double offset;
};

struct OdomIntegratedSensor {
  SHARED(pros::Motor, sensor);
  double offset;
  double gear_ratio;
};

// odometry sensors
// no need to use shared pointers
extern OdomIntegratedSensor odom_left;
extern OdomIntegratedSensor odom_right;
extern OdomSensor odom_middle;
extern SHARED(pros::Imu, inertial);

//// Drivetrain
extern SHARED(pros::Motor, drive_left_front);
extern SHARED(pros::Motor, drive_left_back);
extern SHARED(pros::Motor, drive_left_pto);
extern SHARED(pros::Motor, drive_right_front);
extern SHARED(pros::Motor, drive_right_back);
extern SHARED(pros::Motor, drive_right_pto);

extern pros::MotorGroup drive_left;
extern pros::MotorGroup drive_right;

//// Catapult
extern SHARED(pros::Motor, catapult_motor);
extern SHARED(pros::Rotation, catapult_position);

///// Wings
extern SHARED(pros::adi::Pneumatics, wings);

///// Blocker
extern SHARED(pros::adi::Pneumatics, blocker);

// undefine SHARED macro to prevent accidental use
#undef SHARED