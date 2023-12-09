/**
 * This file just defines the global variables declared in config.hpp.
 * Please edit config.hpp instead of this file for port changes.
 */

#include "config.hpp"

#include "pros/adi.hpp"
#include "pros/rotation.hpp"

#define SHARED(type, name, ...) \
  std::shared_ptr<type> name = std::make_shared<type>(__VA_ARGS__)

SHARED(pros::Motor, drive_left_front, DRIVE_LEFT_FRONT,
       pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_left_back, DRIVE_LEFT_BACK,
       pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_left_pto, -DRIVE_LEFT_PTO, pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_right_front, -DRIVE_RIGHT_FRONT,
       pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_right_back, -DRIVE_RIGHT_BACK,
       pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_right_pto, DRIVE_RIGHT_PTO,
       pros::v5::MotorGear::blue);

OdomIntegratedSensor odom_left = {
    .sensor = drive_left_back,
    .offset = 7.5,
    .gear_ratio = 0.5,
};

OdomIntegratedSensor odom_right = {
    .sensor = drive_right_back,
    .offset = 7.5,
    .gear_ratio = 0.5,
};

OdomSensor odom_middle = {
    .sensor = std::make_shared<pros::adi::Encoder>('c', 'd'),
    .offset = 5,
};

SHARED(pros::Imu, inertial, ODOM_INERTIAL);

std::vector<pros::Motor> drive_left_v = {*drive_left_front, *drive_left_back};

std::vector<pros::Motor> drive_right_v = {*drive_right_front,
                                          *drive_right_back};

SHARED(pros::Motor, catapult_motor, CATAPULT_PORT, pros::v5::MotorGear::red);
SHARED(pros::Rotation, catapult_position, CATAPULT_ROT_PORT);
SHARED(pros::adi::Pneumatics, wings, WINGS_PORT, false);
SHARED(pros::adi::Pneumatics, blocker, BLOCKER_PORT, false);

#undef SHARED