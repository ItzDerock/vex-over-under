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
  catapult::initialize();
  odom::reset();
  odom::initalize();
  static Gif gif("/usd/game.gif", lv_scr_act());
  screen::initAutonSelector(&gif);
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
void autonomous() {
  odom::RobotPosition start = odom::getPosition();

  // SKILLS
  if (odom::autonomous == odom::Autonomous::Skills) {
    odom::moveDistance(-8, 5);
    odom::turnTo(start.theta - M_PI / 2);
    odom::moveDistance(-11, 10);

    pros::delay(5'000);

    odom::moveDistance(9, 10);
    odom::turnTo(start.theta);
    odom::moveDistance(8, 10);
    odom::turnTo(start.theta - M_PI / 4);
    odom::moveDistance(76, 15);

    odom::turnTo(start.theta - M_PI / 2);
    wings->extend();
    odom::moveDistance(30, 10);

    pros::delay(1'000);

    odom::moveDistance(-5, 10);
  } else if (odom::autonomous == odom::Autonomous::Red) {
    wings->toggle();
    odom::moveDistance(32, 3);
    odom::turnTo(start.theta - M_PI / 4);
    odom::moveDistance(10, 3);
    odom::moveDistance(-5, 3);
    odom::turnTo(start.theta - M_PI / 2);
  }

  std::cout << "finished" << std::endl;
}

const double CURVE_SCALE = 6.0;

double driveCurve(double input) {
  // return input;
  return (powf(2.718, -(CURVE_SCALE / 10)) +
          powf(2.718, (fabs(input) - 127) / 10) *
              (1 - powf(2.718, -(CURVE_SCALE / 10)))) *
         input;
}

void move(double left, double right) {
  drive_left_back->move(left);
  drive_left_front->move(left);
  drive_left_pto->move(left);
  drive_right_back->move(right);
  drive_right_front->move(right);
  drive_right_pto->move(right);
}

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
    double throttle = master.get_analog(ANALOG_RIGHT_Y);
    double turn = -1 * master.get_analog(ANALOG_LEFT_X);

    if (throttle == 0) {
      move(driveCurve(throttle + turn), driveCurve(throttle - turn));
    } else {
      double leftPower = throttle + (std::abs(throttle) * turn) / 127.0;
      double rightPower = throttle - (std::abs(throttle) * turn) / 127.0;

      leftPower = driveCurve(leftPower);
      rightPower = driveCurve(rightPower);

      move(leftPower, rightPower);
    }

    // int left = master.get_analog(ANALOG_RIGHT_Y);
    // int right = master.get_analog(ANALOG_LEFT_X);

    // // apply the curve
    // left = driveCurve(left);
    // right = driveCurve(right);

    // // update drive
    // drive_left_back->move(left);
    // drive_left_front->move(left);
    // drive_left_pto->move(left);
    // drive_right_back->move(right);
    // drive_right_front->move(right);
    // drive_right_pto->move(right);

    // if single press, fire once
    if (master.get_digital_new_press(DIGITAL_R1)) {
      catapult::fire();
    } else {
      catapult::rapidFire = master.get_digital(DIGITAL_R1);
    }

    // wings
    if (master.get_digital_new_press(DIGITAL_L1)) {
      wings->toggle();
    }

    // blocker
    if (master.get_digital(DIGITAL_UP)) {
      blocker->move(127);
    } else if (master.get_digital(DIGITAL_DOWN)) {
      blocker->move(-127);
    } else {
      blocker->move(0);
    }

    catapult::ensureTask();

    pros::delay(10);
  }
}