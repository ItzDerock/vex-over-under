#include "main.h"

#include "config.hpp"
#include "gif-pros/gifclass.hpp"
#include "odom/odom.hpp"
#include "pros/misc.h"
#include "screen/screen.hpp"
#include "subsystems/subsystems.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  std::cout << odom_middle.sensor->get_value() << std::endl;

  catapult::initialize();
  odom::initalize();
  odom::reset();
  static Gif gif("/usd/game.gif", lv_scr_act());
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
    // get the joystick values
    int leftJoystick = master.get_analog(ANALOG_RIGHT_Y);
    int rightJoystick = master.get_analog(ANALOG_LEFT_Y);

    // update drive
    drive_left_back->move(leftJoystick);
    drive_left_front->move(leftJoystick);
    drive_left_pto->move(leftJoystick);
    drive_right_back->move(rightJoystick);
    drive_right_front->move(rightJoystick);
    drive_right_pto->move(rightJoystick);

    // if single press, fire once
    if (master.get_digital_new_press(DIGITAL_R1)) {
      catapult::fire();
    }
    // if held, rapid fire
    else if (master.get_digital(DIGITAL_R1)) {
      catapult::rapidFire = true;
    }
    // if released, stop rapid fire
    else {
      catapult::rapidFire = false;
    }

    // wings
    if (master.get_digital_new_press(DIGITAL_L1)) {
      wings->toggle();
    }

    catapult::ensureTask();

    pros::delay(10);
  }
}