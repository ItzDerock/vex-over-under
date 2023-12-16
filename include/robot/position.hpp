#pragma once

namespace odom {

struct RobotPosition {
  double x;
  double y;
  double theta;

  int getDegrees() { return (int)(theta * 180 / M_PI); }
  RobotPosition(double x, double y, double theta) : x(x), y(y), theta(theta) {}

  // subtract operator
  RobotPosition operator-(const RobotPosition& other) {
    return RobotPosition(x - other.x, y - other.y, theta - other.theta);
  }

  // add operator
  RobotPosition operator+(const RobotPosition& other) {
    return RobotPosition(x + other.x, y + other.y, theta + other.theta);
  }

  // multiply operator
  double operator*(const RobotPosition& other) {
    return this->x * other.x + this->y * other.y;
  }

  RobotPosition operator*(const double& other) {
    return RobotPosition(x * other, y * other, theta);
  }

  double distance(const RobotPosition& other) {
    return std::hypot(this->x - other.x, this->y - other.y);
  }

  odom::RobotPosition lerp(odom::RobotPosition other, double t) {
    return odom::RobotPosition(this->x + (other.x - this->x) * t,
                               this->y + (other.y - this->y) * t, this->theta);
  }
};

}