#pragma once

class PIDController
{
public:
  PIDController(double kP, double kI, double kD);
  PIDController(double kP, double kI, double kD, bool debug);
  double update(double error);
  void reset();

private:
  bool debug;
  double _kP, _kI, _kD;
  double _previousError = 0;
  double _integral = 0;
};