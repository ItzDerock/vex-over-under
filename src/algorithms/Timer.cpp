#include "main.h"
#include "robot/utils.hpp"

namespace utils {

/**
 * Implement timer class
 */
Timer::Timer(double duration) {
  startTime = pros::millis();
  endTime = startTime + duration;
}

double Timer::getElapsed() const { return pros::millis() - startTime; }

bool Timer::isUp() const { return pros::millis() >= endTime; }

void Timer::reset() {
  startTime = pros::millis();
  endTime = startTime + endTime - startTime;
}

}  // namespace utils