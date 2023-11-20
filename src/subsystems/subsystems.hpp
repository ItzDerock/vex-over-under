#pragma once

#include "../algorithms/PID.hpp"
#include "pros/rtos.hpp"

#include <memory>

/**
 * Catapult-related subsystem functions
 */
namespace catapult {

#define CATAPULT_ALLOWED_ERROR 10
#define CATAPULT_ZERO_ANGLE 262
#define CATAPULT_READY_STATE CATAPULT_ZERO_ANGLE - 25

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

/**
 * Fires the catapult. Queues if the catapult is not ready.
 */
void fire();

} // namespace catapult

/**
 * Wings-related subsystem functions
 */
namespace wings {}