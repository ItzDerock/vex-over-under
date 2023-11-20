#include "../config.hpp"
#include "subsystems.hpp"
#include <memory>

// simple PD controller for the catapult
std::shared_ptr<PIDController> catapult::catapultPID =
    std::make_shared<PIDController>(1, 0, 0.001);

catapult::CatapultState catapult::catapultState = RELOADING;
pros::Task *catapult::catapultTask = nullptr;
bool catapult::rapidFire = false;

/**
 * A simple state machine-like function to handle catapult control.
 * NOTE: The rotational sensor is BACKWARDS, catapult DOWN = DECREASED ANGLE!
 */
void catapult::update() {
  // if we're ready, do nothing
  // ratchet will keep us in place
  if (catapultState == READY) {
    return;
  }

  // otherwise, calculate error and spin
  double position = (double)catapult_position->get_angle() / 100;
  double error =
      position < CATAPULT_READY_STATE ? CATAPULT_READY_STATE - position : 360;

  // if error is less than the allowed error, we're ready
  // rapidFire needs to be false
  if (error < CATAPULT_ALLOWED_ERROR && catapultState != FIRING && !rapidFire) {
    catapultState = READY;
    catapult_motor->move_velocity(0);
    return;
  }

  // reload if error > 20deg
  if (error > 30 && catapultState == FIRING)
    catapultState = RELOADING;

  double output = catapultPID->update(error);
  catapult_motor->move_velocity(output);
}

void catapult::fire() {
  // if we're ready, fire
  if (catapultState == READY) {
    catapultState = FIRING;
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
  if (catapultTask == nullptr)
    catapultTask = new pros::Task(updateLoop);
}