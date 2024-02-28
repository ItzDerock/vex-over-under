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

  std::cout << "starting" << std::endl;
  odom::MoveToPoseParams pushParams = {
      .chasePower = 100, .slew = 127, .exitOnStall = true};
  odom::MoveToPoseParams pushParamsWithCurve = {.chasePower = 100,
                                                .lead = 1,
                                                .slew = 127,
                                                .forwards = false,
                                                .exitOnStall = true};

  // SKILLS
  switch (odom::autonomous) {
    case odom::Autonomous::Skills: {
      // move to launch position
      odom::moveDistance(-40, 1'000,
                         {.chasePower = 500, .slew = 127, .exitOnStall = true});
      odom::moveDistance(11, 1'000);
      odom::turnTo(71.2);
      odom::moveDistance(-7, 1'000);

      // fire
      wings_left->extend();
      odom::setChassisBrake(pros::E_MOTOR_BRAKE_HOLD);
      odom::holdAngle(71.2);
      odom::move(-20, -20);
      catapult::rapidFire = true;
      catapult::fire();
      // pros::delay(30'000);
      pros::delay(5'000);
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
      odom::startChainedMovement(255);  // chain for rest of time because
                                        // accuracy not super important here.
      odom::moveTo(-33, -63, 90, 2'500, {.chasePower = 5, .lead = 0.55}, false);

      // toggle wings
      odom::moveTo(25, -63, 90, 5'000, {.chasePower = 10, .lead = 0.1}, false);
      odom::turnTo(255);
      odom::moveTo(60, -22, 180, 2'500,
                   {.chasePower = 20,
                    .lead = 0.43,
                    .slew = 127,
                    .forwards = false,
                    .exitOnStall = true},
                   false);  // left push

      odom::moveDistance(12);
      odom::turnTo(120);
      odom::moveDistance(-20);
      wings_left->toggle();

      odom::moveTo(50, -1.3, 270, 3'000, pushParamsWithCurve);
      // odom::moveTo(1, 40, )
      odom::moveDistance(30, 2'500);
      wings_left->toggle();
      odom::turnTo(180);
      odom::moveDistance(-50, 2'500);
      odom::turnTo(300);
      wings_left->toggle();
      odom::moveDistance(-30, 1'500, pushParams);
      odom::moveDistance(30, 2'500);
      wings_left->toggle();
      odom::turnTo(0);
      odom::moveDistance(-30, 2'500);
      odom::turnTo(270);
      wings_left->toggle();
      intake_motor->move(-127);
      odom::moveDistance(-30, 1'500, pushParams);
      odom::moveDistance(30, 2'500);
    } break;

    case odom::Autonomous::SixBall: {
      // activate intake
      blocker_1->toggle();
      blocker_2->toggle();
      pros::delay(300);
      intake_motor->move(127);
      pros::delay(300);
      blocker_1->toggle();
      blocker_2->toggle();
      intake_motor->move(40);

      // descore corner thingy
      // odom::moveDistance(33, 3'000, {.slew = 10});
      odom::moveTo(18.725, -63.547, 90, 3'000,
                   {.chasePower = 10, .lead = 0, .slew = 127});
      odom::turnTo(220, 2'000, 70);  // TODO: ensure only turns right
      wings_left->extend();
      odom::moveTo(
          32, -52.38, 220, 3'000,
          {.chasePower = 20, .lead = 0.3, .slew = 127, .forwards = false});
      odom::turnTo(150);
      wings_left->retract();
      intake_motor->move(-127);
      odom::moveTo(39, -30.38, 180, 3'000,
                   {.chasePower = 20,
                    .lead = 0,
                    .slew = 127,
                    .forwards = false,
                    .exitOnStall = true,
                    .stallThreshold = 2});

      odom::moveDistance(5);
      odom::moveDistance(-5, 1'000,
                         {.chasePower = 500, .slew = 127, .exitOnStall = true});

      // push 2
      wings_left->retract();
      odom::moveDistance(20, 1'000);
      // odom::moveDistance(8, 1'000);
      // odom::turnTo(20);
      // odom::moveDistance(15, 1'000);
      // odom::moveDistance(-14, 1'000);

      // 17,-53
      // 45,40

      // -25, -6

      double angleToBall1 = utils::radToDeg(utils::angleSquish(
          odom::getPosition(false, false).angle({-10, -8, 0})));

      // odom::turnTo(angleToBall1, 2'000);
      intake_motor->move(60);
      // odom::moveDistance(-63, 2'000);
      odom::moveTo(-10, -8, angleToBall1, 2'000,
                   {.chasePower = 10, .slew = 4, .forwards = false}, false);
      odom::turnTo(270, 2'500, 60);
      wings_left->extend();
      intake_motor->move(-127);
      odom::moveDistance(-30, 1'000, {.slew = 127, .exitOnStall = true});
      odom::moveDistance(5, 1'000);

      // -22, -44
      // double angleToPole = utils::radToDeg(utils::angleSquish(
      //     odom::getPosition(false, false).angle({-22, -46, 0})));

      intake_motor->move(0);
      odom::turnTo(216, 2'000);
      wings_left->retract();
      blocker_1->extend();
      blocker_2->extend();
      odom::moveDistance(50, 1'000);

    } break;

    case odom::Autonomous::TouchBar: {
      odom::moveDistance(-12, 1'000,
                         {.maxSpeed = 80, .chasePower = 5, .slew = 5});
    } break;

    case odom::Autonomous::Defense: {
      // TODO
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
      blocker_1->toggle();
      blocker_2->toggle();
    }

    // blocker
    if (master.get_digital_new_press(DIGITAL_L1)) {
      wings_left->toggle();
    }

    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      dtReversed = !dtReversed;
    }

    catapult::ensureTask();

    pros::delay(10);
  }
}
