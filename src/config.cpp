/**
 * This file just defines the global variables declared in config.hpp.
 * Please edit config.hpp instead of this file for port changes.
 */

#include "config.hpp"

#include "pros/adi.hpp"
#include "pros/rotation.hpp"

#define SHARED(type, name, ...) \
  std::shared_ptr<type> name = std::make_shared<type>(__VA_ARGS__)

SHARED(pros::Motor, drive_left_front, DRIVE_LEFT_FRONT, DRIVETRAIN_GEARBOX);
SHARED(pros::Motor, drive_left_back, DRIVE_LEFT_BACK, DRIVETRAIN_GEARBOX);
SHARED(pros::Motor, drive_left_pto, DRIVE_LEFT_PTO, DRIVETRAIN_GEARBOX);
SHARED(pros::Motor, drive_right_front, -DRIVE_RIGHT_FRONT, DRIVETRAIN_GEARBOX);
SHARED(pros::Motor, drive_right_back, -DRIVE_RIGHT_BACK, DRIVETRAIN_GEARBOX);
SHARED(pros::Motor, drive_right_pto, -DRIVE_RIGHT_PTO, DRIVETRAIN_GEARBOX);

// can't use a designator until P2287R1 is merged
// https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2021/p2287r1.html
// which won't make it to C++23
// offset, gear ratio, wheel size
OdomIntegratedSensor odom_left(drive_left_back, DRIVE_TRACK_WIDTH / 2,
                               DRIVETRAIN_GEAR_RATIO, 4);
OdomIntegratedSensor odom_right(drive_right_back, DRIVE_TRACK_WIDTH / 2,
                                DRIVETRAIN_GEAR_RATIO, 4);
OdomSensor odom_middle(std::make_shared<pros::adi::Encoder>(ODOM_MIDDLE_PORT,
                                                            true),
                       4, 1, ODOMETRY_WHEEL_DIAMETER);

SHARED(pros::Imu, inertial, ODOM_INERTIAL);

SHARED(pros::Motor, catapult_motor, CATAPULT_PORT, pros::v5::MotorGear::red);
SHARED(pros::Rotation, catapult_position, CATAPULT_ROT_PORT);
SHARED(pros::adi::Pneumatics, wings, WINGS_PORT, false);
SHARED(pros::adi::Pneumatics, blocker, BLOCKER_PORT, false);
SHARED(pros::Motor, intake_motor, INTAKE_PORT, pros::v5::MotorGear::blue);

#undef SHARED