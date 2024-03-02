#include <memory>

#include "../config.hpp"
#include "robot/subsystems.hpp"

// simple PD controller for the catapult
std::shared_ptr<PIDController> catapult::catapultPID =
    std::make_shared<PIDController>(1, 0, 0.001);

catapult::CatapultState catapult::catapultState = RELOADING;
pros::Task *catapult::catapultTask = nullptr;
bool catapult::rapidFire = false;

#define FIRE_SPEED 140

/**
 * A simple state machine-like function to handle catapult control.
 * NOTE: The rotational sensor is BACKWARDS, catapult DOWN = DECREASED ANGLE!
 */
void catapult::update() {
  // if we're ready, do nothing
  // ratchet will keep us in place
  // if (catapultState == READY) {
  //   return;
  // }

  // otherwise, calculate error and spin
  double position = (double)catapult_position->get_angle() / 100;
  double error =
      position < CATAPULT_READY_STATE ? CATAPULT_READY_STATE - position : 360;

  if (catapultState == READY) {
    if (position > CATAPULT_READY_STATE + CATAPULT_ALLOWED_ERROR) {
      catapultState = RELOADING;
      catapult_motor->move(FIRE_SPEED);
    } else {
      catapult_motor->move(0);
      return;
    }
  }

  // if error is less than the allowed error, we're ready
  // rapidFire needs to be false
  if (error < CATAPULT_ALLOWED_ERROR && catapultState != FIRING && !rapidFire) {
    catapultState = READY;
    catapult_motor->move(0);
    return;
  }

  // reload if error > 20deg
  if (error > 30 && catapultState == FIRING) catapultState = RELOADING;

  catapult_motor->move(FIRE_SPEED);
}

void catapult::fire() {
  // if we're ready, fire
  if (catapultState == READY) {
    catapult_motor->move(FIRE_SPEED);

    pros::Task([] {
      pros::delay(300);
      catapultState = FIRING;
    });
  }

  printf("[warn] catapult is not ready to fire! Current state: %d\n",
         catapultState);
}

void catapult::updateLoop() {
  while (true) {
    update();
    pros::delay(10);
  }
}

void catapult::initialize() {
  if (catapultTask == nullptr) {
    catapultTask = new pros::Task(updateLoop);
    catapultTask->resume();
  }
}

void catapult::ensureTask() {
  if (catapultTask == nullptr) {
    printf("[warn] catapult task is null, initializing\n");
    initialize();
  }

  auto state = catapultTask->get_state();

  if (state == pros::E_TASK_STATE_SUSPENDED ||
      state == pros::E_TASK_STATE_INVALID) {
    printf("[warn] catapult task is not running, currently is %d\n", state);
    catapultTask->resume();
  }
}