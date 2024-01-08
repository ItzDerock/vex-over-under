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
  odom::loadPaths(
      {"/usd/skills/push-left.txt", /*"/usd/skills/push-center.txt"*/});

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
      // TURN TEST
      // odom::turnTo(90);
      // pros::delay(1000);
      // odom::turnTo(180);
      // pros::delay(1000);
      // odom::turnTo(270);
      // pros::delay(1000);
      // odom::turnTo(0);

      // SIMPLE MOVE TEST
      // odom::moveTo(26, 0, 0, 10'000, {.lead = 0}, false);
      // pros::delay(5'000);
      // odom::moveTo(0, 0, 0, 10'000, {.forwards = false}, false);

      odom::moveTo(0, 25, 0, 10'000, {.lead = 0}, false);
      odom::moveTo(12, 20, 270, 10'000, {.lead = 0, .forwards = false}, false);
      pros::delay(5'000);
      odom::moveTo(0, 0, 180, 10'000,
                   {.chasePower = 0.5, .lead = 0.8, .slew = 5}, false);

      // false); intake_motor->move(0); odom::moveTo(0, 0, 0, 10'000, {.lead =
      // 0.8, .forwards = false}, false); odom::moveTo(25, -22, 270, 10'000,
      //              {.chasePower = 0.5, .lead = 0.75, .slew = 5}, false);
      break;

      // case odom::Autonomous::ScoreLeft:
      //   // wings->toggle();
      //   odom::moveDistance(32, 3);
      //   // odom::turnTo(start.theta - M_PI / 4);
      //   for (int i = 0; i < 3; i++) {
      //     odom::moveDistance(10, 10);
      //     odom::moveDistance(-10, 10);
      //   }
      //   break;
      //   // odom::turnTo(start.theta - M_PI / 2);

      // case odom::Autonomous::ScoreSimple:
      //   odom::moveDistance(64, 5);
      //   odom::moveDistance(-8, 5);
      //   break;

      // case odom::Autonomous::TouchBar:
      //   blocker->toggle();
      //   odom::moveDistance(-21, 5);
      //   blocker->toggle();
      //   break;

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
    if (master.get_digital_new_press(DIGITAL_L1)) {
      wings->toggle();
    }

    // blocker
    if (master.get_digital_new_press(DIGITAL_R2)) {
      blocker->toggle();
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