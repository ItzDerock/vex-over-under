/**
 * This file just defines the global variables declared in config.hpp.
 * Please edit config.hpp instead of this file for port changes.
 */

#include "config.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#define SHARED(type, name) std::shared_ptr<type> name

SHARED(pros::Rotation,
       odom_left_sensor) = std::make_shared<pros::Rotation>(ODOM_LEFT_PORT);

SHARED(pros::Rotation,
       odom_right_sensor) = std::make_shared<pros::Rotation>(ODOM_RIGHT_PORT);

SHARED(pros::Rotation,
       odom_middle_sensor) = std::make_shared<pros::Rotation>(ODOM_MIDDLE_PORT);

OdomSensor odom_left = {
    .sensor = odom_left_sensor,
    .offset = 0,
};

OdomSensor odom_right = {
    .sensor = odom_right_sensor,
    .offset = 0,
};

OdomSensor odom_middle = {
    .sensor = odom_middle_sensor,
    .offset = 0,
};

SHARED(pros::Motor, drive_left_front) =
    std::make_shared<pros::Motor>(-DRIVE_LEFT_FRONT, pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_left_back) =
    std::make_shared<pros::Motor>(-DRIVE_LEFT_BACK, pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_left_pto) =
    std::make_shared<pros::Motor>(DRIVE_LEFT_PTO, pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_right_front) =
    std::make_shared<pros::Motor>(DRIVE_RIGHT_FRONT, pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_right_back) =
    std::make_shared<pros::Motor>(DRIVE_RIGHT_BACK, pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_right_pto) =
    std::make_shared<pros::Motor>(-DRIVE_RIGHT_PTO, pros::v5::MotorGear::blue);

std::vector<pros::Motor> drive_left_v = {*drive_left_front, *drive_left_back};

std::vector<pros::Motor> drive_right_v = {*drive_right_front,
                                          *drive_right_back};

SHARED(pros::Motor, catapult_motor) =
    std::make_shared<pros::Motor>(CATAPULT_PORT, pros::v5::MotorGear::red);

SHARED(pros::Rotation,
       catapult_position) = std::make_shared<pros::Rotation>(CATAPULT_ROT_PORT);

SHARED(pros::adi::Pneumatics,
       wings) = std::make_shared<pros::adi::Pneumatics>(WINGS_PORT, false);

#undef SHARED