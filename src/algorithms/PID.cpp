#include "robot/PID.hpp"

#include <cstdio>

PIDController::PIDController(double kP, double kI, double kD)
    : _kP(kP), _kI(kI), _kD(kD), debug(false) {}

PIDController::PIDController(double kP, double kI, double kD, bool debug)
    : _kP(kP), _kI(kI), _kD(kD), debug(debug) {}

double PIDController::update(double error) {
  _integral += error;
  double derivative = error - _previousError;

  double kPOutput = _kP * error;
  double kIOutput = _kI * _integral;
  double kDOutput = _kD * derivative;

  if (debug)
    printf("err: %f, kP: %f, kI: %f, kD: %f\n", error, kPOutput, kIOutput,
           kDOutput);

  double output = kPOutput + kIOutput + kDOutput;

  _previousError = error;
  return output;
}

void PIDController::reset() {
  _previousError = 0;
  _integral = 0;

  if (debug) printf("PID reset");
}