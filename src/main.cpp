#include "main.h"

#include "config.hpp"
#include "gif-pros/gifclass.hpp"
#include "pros-mpeg/mpeg.hpp"
#include "pros/misc.h"
#include "robot/odom.hpp"
#include "robot/screen.hpp"
#include "robot/subsystems.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  catapult::initialize();

  odom::initalize();
  odom::reset({0, 0, 0});

  // load pure pursuit paths
  odom::loadPaths({"/usd/skills/push-left.txt",
                   "/usd/pathtest.txt" /*"/usd/skills/push-center.txt"*/});

  static MPEGPlayer mpeg("/usd/game.mpeg", lv_scr_act());
  screen::initAutonSelector(&mpeg);
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

  std::cout << "starting" << std::endl;

  // SKILLS
  switch (odom::autonomous) {
    case odom::Autonomous::Skills:
      // move to launch position
      odom::moveDistance(-40, 1'000);
      odom::moveDistance(8, 1'000);
      odom::turnTo(68);
      odom::moveDistance(-8, 1'000);
      // odom::moveDistance(-9, 5'000);
      // odom::turnTo(60);
      // odom::moveDistance(-7.5, 2'500);

      // fire
      odom::move(-5, -5);
      catapult::rapidFire = true;
      catapult::fire();
      pros::delay(30'000);
      catapult::rapidFire = false;
      odom::move(0, 0);

      // move to other side
      odom::moveTo(-33, -65, 90, 2'500, {.chasePower = 5, .lead = 0.45}, false);
      wings->toggle();
      wings_2->toggle();
      pros::delay(300);
      wings->toggle();
      wings_2->toggle();

      // toggle wings
      odom::moveTo(25, -65, 90, 5'000, {.chasePower = 10, .lead = 0.1}, false);
      odom::moveTo(53.46, -28.219, 0, 2'500, {.chasePower = 15, .lead = 0.40},
                   false);

      odom::moveDistance(-8);
      odom::turnTo(115);
      odom::moveDistance(-40, 5'000);
      blocker->toggle();
      odom::turnTo(235);
      odom::moveDistance(-30, 1'500);
      odom::moveDistance(30, 2'500);
      blocker->toggle();
      odom::turnTo(180);
      odom::moveDistance(-50, 2'500);
      odom::turnTo(300);
      blocker->toggle();
      odom::moveDistance(-30, 1'500);
      odom::moveDistance(30, 2'500);
      blocker->toggle();
      odom::turnTo(0);
      odom::moveDistance(-30, 2'500);
      odom::turnTo(270);
      blocker->toggle();
      odom::moveDistance(-30, 1'500);
      odom::moveDistance(30, 2'500);
      break;

    case odom::Autonomous::ScoreLeft:
      // activate intake
      wings->toggle();
      wings_2->toggle();
      pros::delay(300);
      intake_motor->move(127);
      pros::delay(300);
      wings->toggle();
      wings_2->toggle();
      intake_motor->move(40);

      // descore corner thingy
      odom::moveDistance(33, 3'000, 10);
      odom::turnTo(225);  // TODO: ensure only turns right
      blocker->toggle();
      odom::moveDistance(-23, 1'000);

      // push goal
      intake_motor->move(-127);
      odom::turnTo(180);
      odom::moveDistance(-15, 1'000);

      // push 2
      blocker->retract();
      odom::moveDistance(14, 1'000);
      // odom::moveDistance(8, 1'000);
      // odom::turnTo(20);
      // odom::moveDistance(15, 1'000);
      // odom::moveDistance(-14, 1'000);

      // grab mid bot
      odom::turnTo(116);
      intake_motor->move(60);
      odom::moveDistance(-45, 2'000);
      pros::delay(100);
      odom::turnTo(235);
      intake_motor->move(-90);
      odom::moveDistance(-5, 1'000);
      pros::delay(100);

      // ball 2
      intake_motor->move(60);
      odom::turnTo(151);
      odom::moveDistance(-23, 1'000);
      odom::turnTo(270);
      intake_motor->move(-40);

      // push
      blocker->extend();
      odom::moveDistance(-30, 1'000);
      odom::moveDistance(10, 1'000);

      // pros::delay(500);

      // intake_motor->move(70);
      // odom::turnTo(161);
      // odom::moveTo(-29, -16, 161, 2'000,
      //              {.chasePower = 10, .lead = 0.35, .forwards = false},
      //              false);

      // odom::turnTo(270);
      // intake_motor->move(-40);
      // odom::moveDistance(-20, 1'000);

      // 200deg

      // odom::moveTo(38.6, -34.7, 184, 2'000, {.chasePower = 10, .lead = 0.35},
      //              false);

      // odom::moveTo(30.8, -43.5, 180, 3'000,
      //              {.chasePower = 14, .lead = 0.35, .forwards = false},
      //              false);
      // intake_motor->move(-127);
      // odom::moveDistance(-16, 1'000);
      // odom::moveDistance(8, 1'000);
      // odom::turnTo(180);
      // odom::moveDistance(-8, 1'500);

      // blocker->toggle();

      // // grab triball 1
      // odom::turnTo(90 - 20);  // todo: change this? - face triball
      // // todo: intake
      // odom::moveDistance(-57);
      // odom::turnTo(60);  // todo: face goal
      // // todo: outtake

      // // odom::moveDistance(-5, 1'000);
      // // odom::moveDistance(5, 1'000);

      // // grab triball 2
      // odom::turnTo(180 - 20);  // face triball
      // // todo: intake
      // odom::moveDistance(-26, 1'000);
      // odom::turnTo(90);  // face goal
      // blocker->toggle();
      // // todo: outtake
      // odom::moveDistance(30, 1'500);
      // odom::moveDistance(-10, 1'000);

      // todo: touch bar
    default:
      break;
  }
}

const double CURVE_SCALE = 6.0;

double driveCurve(double input) {
  // return input;
  return (powf(2.718, -(CURVE_SCALE / 10)) +
          powf(2.718, (fabs(input) - 127) / 10) *
              (1 - powf(2.718, -(CURVE_SCALE / 10)))) *
         input;
}

// static void move(double left, double right) {
//   drive_left_back->move(left);
//   drive_left_front->move(left);
//   drive_left_pto->move(left);
//   drive_right_back->move(right);
//   drive_right_front->move(right);
//   drive_right_pto->move(right);
// }

/**
 * Runs the operator control code. This function will be started in its own
 * task with the default priority and stack size whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the
 * operator control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart
 * the task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  bool cataWasHeld = false;
  bool dtReversed = false;

  while (true) {
    // get the joystick values
    double throttle = master.get_analog(ANALOG_RIGHT_Y);
    double turn = master.get_analog(ANALOG_LEFT_X);

    // small deadzone
    if (fabs(throttle) < 2) {
      double leftPower = driveCurve(turn);

      odom::move(leftPower, -leftPower);
    } else {
      double leftPower = throttle + (std::abs(throttle) * turn) / 127.0;
      double rightPower = throttle - (std::abs(throttle) * turn) / 127.0;

      leftPower = driveCurve(leftPower);
      rightPower = driveCurve(rightPower);

      if (dtReversed) {
        double tmp = leftPower;
        leftPower = -rightPower;
        rightPower = -tmp;
        // leftPower *= -1;
        // rightPower *= -1;
      }

      odom::move(leftPower, rightPower);
    }

    // if single press, fire once
    if (master.get_digital_new_press(DIGITAL_X)) {
      catapult::fire();
      cataWasHeld = true;
    } else if (cataWasHeld) {
      catapult::rapidFire = master.get_digital(DIGITAL_X);

      if (catapult::rapidFire == false) {
        cataWasHeld = false;
      }
    }

    // r2 for toggle
    if (master.get_digital_new_press(DIGITAL_A)) {
      catapult::rapidFire = !catapult::rapidFire;
      if (catapult::rapidFire) {
        catapult::fire();
      }
    }

    // intake
    intake_motor->move(master.get_digital(DIGITAL_R1) * 127 -
                       master.get_digital(DIGITAL_R2) * 127);

    // wings
    if (master.get_digital_new_press(DIGITAL_L2)) {
      wings->toggle();
      wings_2->toggle();
    }

    // blocker
    if (master.get_digital_new_press(DIGITAL_L1)) {
      blocker->toggle();
    }

    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      dtReversed = !dtReversed;
    }

    // blocker
    // if (master.get_digital(DIGITAL_UP)) {
    //   blocker->move(127);
    // } else if (master.get_digital(DIGITAL_DOWN)) {
    //   blocker->move(-127);
    // } else {
    //   blocker->move(0);
    // }

    catapult::ensureTask();

    pros::delay(10);
  }
}