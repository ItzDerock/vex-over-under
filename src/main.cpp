#include "main.h"

#include "config.hpp"
#include "pros-mpeg/mpeg.hpp"
#include "pros/misc.h"
#include "robot/odom.hpp"
#include "robot/screen.hpp"
#include "robot/subsystems.hpp"
#include "robot/utils.hpp"

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

  const odom::MoveToPoseParams pushParams = {
      .chasePower = 100, .slew = 127, .exitOnStall = true};
  const odom::MoveToPoseParams pushParamsWithCurve = {.chasePower = 100,
                                                      .lead = 0.3,
                                                      .slew = 127,
                                                      .forwards = false,
                                                      .exitOnStall = true};

  // SKILLS
  switch (odom::autonomous) {
    case odom::Autonomous::Skills: {
      // move to launch position
      odom::moveTo(-54, -30, 180, 2'000,
                   {.chasePower = 127,
                    .lead = 0.3,
                    .slew = 127,
                    .forwards = false,
                    .exitOnStall = true});
      odom::moveDistance(11, 1'000);
      odom::turnTo(69.2);
      odom::startChainedMovement(1);
      odom::moveDistance(-7, 1'000);
      wings_left->extend();

      // fire
      odom::setChassisBrake(pros::E_MOTOR_BRAKE_HOLD);
      odom::holdAngle(69.2);
      odom::move(-20, -20);
      catapult::rapidFire = true;
      catapult::fire();
      pros::delay(30'000);
      // pros::delay(5'000);
      catapult::rapidFire = false;
      odom::move(0, 0);
      odom::setChassisBrake(pros::E_MOTOR_BRAKE_COAST);
      odom::holdAngle(-1);

      // move to other side
      pros::Task([]() {
        wings_left->retract();
        pros::delay(400);
        blocker_1->toggle();
        blocker_2->toggle();
        pros::delay(300);
        blocker_1->toggle();
        blocker_2->toggle();
      });

      odom::moveTo(-33, -63, 90, 2'500, {.chasePower = 5, .lead = 0.55}, false);

      // toggle wings
      odom::moveTo(25, -63, 90, 5'000, {.chasePower = 10, .lead = 0.1}, false);
      odom::turnTo(255);
      wings_left->extend();
      odom::moveTo(62, -22, 180, 2'500,
                   {.chasePower = 20,
                    .lead = 0.43,
                    .slew = 127,
                    .forwards = false,
                    .exitOnStall = true},
                   false);  // left push

      odom::moveDistance(6);
      wings_left->retract();
      odom::turnTo(270);
      odom::startChainedMovement(255);  // chain for rest of time because
                                        // accuracy not super important here.
      odom::moveTo(12, -28, 270, 2'500, {.chasePower = 10, .lead = 0}, false);
      odom::turnTo(210);
      wings_left->extend();
      wings_right->extend();
      odom::moveTo(42, -6, 270, 2'500, pushParamsWithCurve, false);

      // back up for push 2
      odom::moveDistance(5, 1'000);
      wings_left->retract();
      wings_right->retract();
      odom::moveDistance(10, 1'000);

      // position for push 2
      odom::moveTo(10, 40, 0, 5'000,
                   {.chasePower = 20, .lead = 0.3, .exitOnStall = true}, false);
      odom::turnTo(325);

      // push!
      wings_left->extend();
      wings_right->extend();
      odom::moveTo(45, 4, 270, 5'000, pushParamsWithCurve, false);
      odom::moveDistance(5, 1'000);
      wings_left->retract();
      wings_right->retract();

      // last stretch
      odom::moveTo(12, 10, 270, 2'000,
                   {.chasePower = 20, .lead = 0.3, .forwards = false}, false);
      odom::turnTo(212);
      wings_left->extend();
      wings_right->extend();
      odom::moveTo(39, 43, 250, 5'000,
                   {.chasePower = 20, .lead = 0.4, .forwards = false}, false);
      wings_right->retract();
      odom::moveTo(60, 40, 0, 5'000,
                   {.chasePower = 20,
                    .lead = 0.3,
                    .forwards = false,
                    .exitOnStall = true},
                   false);
      odom::moveDistance(-25, 1'000, pushParams);
      odom::moveDistance(5, 1'000);
      wings_left->retract();

    } break;

    case odom::Autonomous::SixBall: {
      // Gets the ball under the intake and releases intake from stored position
      blocker_1->toggle();
      blocker_2->toggle();
      pros::delay(200);
      intake_motor->move(127);
      pros::delay(550);
      blocker_1->toggle();
      blocker_2->toggle();
      intake_motor->move(40);
      // ---

      // Lines up to descore corner ball
      odom::moveTo(18.725, -63.547, 90, 3'000,
                   {.chasePower = 10, .lead = 0, .slew = 127});
      odom::turnTo(220, 2'000, 70);  // TODO: ensure only turns right
      // ----

      // Descores corner ball
      wings_right->extend();
      odom::startChainedMovement(5);
      odom::moveTo(32.5, -52.38, 220, 3'000,
                   {.chasePower = 30,
                    .lead = 0.3,
                    .slew = 127,
                    .forwards = false,
                    .exitOnStall = true});  // CHAINED 1
      // small turn to ensure descored
      odom::turnTo(150);  // CHAINED 2
      // ---

      // Does a push to ensure all balls scored in goal
      wings_right->retract();
      wings_left->extend();
      intake_motor->move(-127);
      odom::moveTo(39, -30.38, 180, 3'000,
                   {.chasePower = 20,
                    .lead = 0,
                    .slew = 127,
                    .forwards = false,
                    .exitOnStall = true,
                    .stallThreshold = 2});  // CHAINED 3

      odom::moveDistance(5);                      // CHAINED 4
      odom::moveDistance(-5, 1'000, pushParams);  // CHAINED 5
      wings_left->retract();
      // ---

      // line up for middle ball
      odom::moveDistance(20, 1'000);

      // go to ball
      double angleToBall1 = utils::radToDeg(utils::angleSquish(
          odom::getPosition(false, false).angle({-10, -8, 0})));

      intake_motor->move(60);
      odom::moveTo(-10, -8, angleToBall1, 2'000,
                   {.chasePower = 10, .slew = 4, .forwards = false}, false);
      odom::turnTo(270, 2'500, 60);
      wings_left->extend();
      wings_right->extend();
      intake_motor->move(-127);
      odom::moveDistance(-30, 1'000, {.slew = 127, .exitOnStall = true});
      odom::moveDistance(5, 1'000);
      intake_motor->move(0);
      odom::turnTo(216, 2'000);
      wings_left->retract();
      wings_right->retract();

      // touch bar
      blocker_1->extend();
      blocker_2->extend();
      odom::moveDistance(50, 1'000);  // TODO: absolute

    } break;

    case odom::Autonomous::TouchBar: {
      odom::moveDistance(-12, 1'000,
                         {.maxSpeed = 80, .chasePower = 5, .slew = 5});
    } break;

    case odom::Autonomous::Defense: {
      // nav to middle ball
      odom::startChainedMovement(2);
      odom::moveDistance(-6, 500);
      odom::moveTo(
          -55, 0, 196, 5'000,
          {.chasePower = 5000, .lead = -0.15, .slew = 127, .forwards = false},
          true);

      // release intake
      blocker_1->extend();
      blocker_2->extend();
      pros::delay(100);
      intake_motor->move(127);
      pros::delay(200);
      blocker_1->retract();
      blocker_2->retract();
      odom::waitUntilSettled(5'000);
      pros::delay(100);
      // intake_motor->move(60 /);

      // go back to start
      odom::moveTo(-88, -28, 305, 2'000, {.lead = 0.3});
      wings_right->extend();
      intake_motor->move(-30);
      odom::moveDistance(-20, 2'000, {.exitOnStall = true});
      odom::turnTo(270);
      wings_right->retract();

      intake_motor->move(-60);
      odom::moveTo(-36, -41, 270, 5'000,
                   {.chasePower = 8, .lead = 0.1, .forwards = false});
      blocker_1->extend();
      blocker_2->extend();
      wings_left->extend();
      pros::delay(500);
      blocker_1->retract();
      blocker_2->retract();
      odom::turnTo(285);

    } break;

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
    if (fabs(throttle) < 4) {
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
    intake_motor->move(
        master.get_digital(DIGITAL_R1) * 127 -
        (master.get_digital(DIGITAL_L1) || master.get_digital(DIGITAL_R2)) *
            127);

    // wings
    // if (master.get_digital_new_press(DIGITAL_R2) ||
    if (master.get_digital_new_press(DIGITAL_L2)) {
      bool extended = wings_left->is_extended() || wings_right->is_extended();

      if (extended) {
        // retract
        wings_left->retract();
        wings_right->retract();
      } else {
        wings_left->extend();
        wings_right->extend();
      }
    }

    // blocker
    if (master.get_digital_new_press(DIGITAL_UP)) {
      blocker_1->toggle();
      blocker_2->toggle();
    }

    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      dtReversed = !dtReversed;
      master.rumble(dtReversed ? "-" : ".");
    }

    catapult::ensureTask();

    pros::delay(10);
  }
}
