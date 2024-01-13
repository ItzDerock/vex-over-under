#pragma once

#include <memory>

#include "PID.hpp"
#include "pros/rtos.hpp"

/**
 * Catapult-related subsystem functions
 */
namespace catapult {

#define CATAPULT_ALLOWED_ERROR 8
#define CATAPULT_ZERO_ANGLE 350
#define CATAPULT_READY_STATE 330

enum CatapultState { READY, RELOADING, FIRING };
extern CatapultState catapultState;
extern bool rapidFire;

extern std::shared_ptr<PIDController> catapultPID;
extern pros::Task *catapultTask;

/**
 * Ticks the catapult subsystem.
 */
void update();

/**
 * Runs the catapult subsystem.
 * Should be run in a separate pros task, as it is blocking.
 */
void updateLoop();

/**
 * Starts the catapult subsystem.
 */
void initialize();
void ensureTask();

/**
 * Fires the catapult. Queues if the catapult is not ready.
 */
void fire();

}  // namespace catapult