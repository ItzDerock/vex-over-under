#include "config.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
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
    std::make_shared<pros::Motor>(DRIVE_LEFT_FRONT, pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_left_back) =
    std::make_shared<pros::Motor>(DRIVE_LEFT_BACK, pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_left_pto) =
    std::make_shared<pros::Motor>(DRIVE_LEFT_PTO, pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_right_front) =
    std::make_shared<pros::Motor>(DRIVE_RIGHT_FRONT, pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_right_back) =
    std::make_shared<pros::Motor>(DRIVE_RIGHT_BACK, pros::v5::MotorGear::blue);
SHARED(pros::Motor, drive_right_pto) =
    std::make_shared<pros::Motor>(DRIVE_RIGHT_PTO, pros::v5::MotorGear::blue);

std::vector<pros::Motor> drive_left_v = {*drive_left_front, *drive_left_back};

std::vector<pros::Motor> drive_right_v = {*drive_right_front,
                                          *drive_right_back};

// pros::MotorGroup drive_left(drive_left_v);

// pros::MotorGroup drive_left({*drive_bottom_right, *drive_bottom_right});
// pros::MotorGroup drive_right({*drive_right_front, *drive_right_back});

// auto drive_left = pros::MotorGroup({*drive_left_front, *drive_left_back});
// SHARED(pros::MotorGroup, drive_right) = std::make_shared<pros::MotorGroup>(

#undef SHARED