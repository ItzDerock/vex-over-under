// The implementation below is mostly based off of
// the document written by Dawgma
// Here is a link to the original document
// https://www.chiefdelphi.com/uploads/default/original/3X/b/e/be0e06de00e07db66f97686505c3f4dde2e332dc.pdf

// Code inspired by Lemlib's implemenation
// slight modifications made to optimize performance and make it work with our
// custom lib

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include "./odom.hpp"
#include "main.h"
#include "pros/misc.hpp"
#include "robot/utils.hpp"

int distTraveled = 0;
pros::Mutex mutex();

/**
 * @brief function that returns elements in a file line, separated by a
 * delimeter
 *
 * @param input the raw string
 * @param delimeter string separating the elements in the line
 * @return std::vector<std::string> array of elements read from the file
 */
static std::vector<std::string> readElement(const std::string& input,
                                            std::string delimiter) {
  std::string token;
  std::string s = input;
  std::vector<std::string> output;
  size_t pos = 0;

  // main loop
  while ((pos = s.find(delimiter)) !=
         std::string::npos) {  // while there are still delimiters in the string
    token = s.substr(0, pos);  // processed substring
    output.push_back(token);
    s.erase(0, pos + delimiter.length());  // remove the read substring
  }

  output.push_back(s);  // add the last element to the returned string

  return output;
}

/**
 * @brief Get a path from the sd card
 *
 * @param filePath The file to read from
 * @return std::vector<odom::RobotPosition> vector of points on the path
 */
static std::vector<odom::RobotPosition> getData(const std::string& path) {
  std::vector<odom::RobotPosition> robotPath;
  std::string line;
  std::vector<std::string> pointInput;
  odom::RobotPosition pathPoint(0, 0, 0);

  // read path
  std::ostringstream buf;
  std::ifstream input(path);
  buf << input.rdbuf();
  std::string data = buf.str();

  // format data from the asset
  std::vector<std::string> dataLines = readElement(data, "\n");

  // read the points until 'endData' is read
  for (std::string line : dataLines) {
    if (line == "endData" || line == "endData\r") break;
    pointInput = readElement(line, ", ");           // parse line
    pathPoint.x = std::stof(pointInput.at(0));      // x position
    pathPoint.y = std::stof(pointInput.at(1));      // y position
    pathPoint.theta = std::stof(pointInput.at(2));  // velocity
    robotPath.push_back(pathPoint);                 // save data
  }

  return robotPath;
}

/**
 * @brief find the closest point on the path to the robot
 *
 * @param pose the current pose of the robot
 * @param path the path to follow
 * @return int index to the closest point
 */
int findClosest(odom::RobotPosition pose,
                std::vector<odom::RobotPosition> path) {
  int closestPoint;
  float closestDist = 1000000;
  float dist;

  // loop through all path points
  for (int i = 0; i < path.size(); i++) {
    dist = pose.distance(path.at(i));
    if (dist < closestDist) {  // new closest point
      closestDist = dist;
      closestPoint = i;
    }
  }

  return closestPoint;
}

/**
 * @brief Function that finds the intersection point between a circle and a line
 *
 * @param p1 start point of the line
 * @param p2 end point of the line
 * @param pos position of the robot
 * @param path the path to follow
 * @return float how far along the line the
 */
float circleIntersect(odom::RobotPosition p1, odom::RobotPosition p2,
                      odom::RobotPosition pose, float lookaheadDist) {
  // calculations
  // uses the quadratic formula to calculate intersection points
  odom::RobotPosition d = p2 - p1;
  odom::RobotPosition f = p1 - pose;
  float a = d * d;
  float b = 2 * (f * d);
  float c = (f * f) - lookaheadDist * lookaheadDist;
  float discriminant = b * b - 4 * a * c;

  // if a possible intersection was found
  if (discriminant >= 0) {
    discriminant = sqrt(discriminant);
    float t1 = (-b - discriminant) / (2 * a);
    float t2 = (-b + discriminant) / (2 * a);

    // prioritize further down the path
    if (t2 >= 0 && t2 <= 1)
      return t2;
    else if (t1 >= 0 && t1 <= 1)
      return t1;
  }

  // no intersection found
  return -1;
}

/**
 * @brief returns the lookahead point
 *
 * @param lastLookahead - the last lookahead point
 * @param pose - the current position of the robot
 * @param path - the path to follow
 * @param lookaheadDist - the lookahead distance of the algorithm
 */
odom::RobotPosition lookaheadPoint(odom::RobotPosition lastLookahead,
                                   odom::RobotPosition pose,
                                   std::vector<odom::RobotPosition> path,
                                   float lookaheadDist) {
  // find the furthest lookahead point on the path

  // optimizations applied:
  // - made the starting index the one after lastLookahead's index,
  // as anything before would be discarded
  // - searched the path in reverse, as the first hit would be
  // the guaranteed farthest lookahead point
  for (int i = path.size() - 1; i > lastLookahead.theta; i--) {
    // since we are searching in reverse, instead of getting
    // the current pose and the next one, we should get the
    // current pose and the *last* one
    odom::RobotPosition lastPathPose = path.at(i - 1);
    odom::RobotPosition currentPathPose = path.at(i);

    float t =
        circleIntersect(lastPathPose, currentPathPose, pose, lookaheadDist);

    if (t != -1) {
      odom::RobotPosition lookahead = lastPathPose.lerp(currentPathPose, t);
      lookahead.theta = i;
      return lookahead;
    }
  }

  // robot deviated from path, use last lookahead point
  return lastLookahead;
}

/**
 * @brief Get the curvature of a circle that intersects the robot and the
 * lookahead point
 *
 * @param pos the position of the robot
 * @param heading the heading of the robot
 * @param lookahead the lookahead point
 * @return float curvature
 */
float findLookaheadCurvature(odom::RobotPosition pose, float heading,
                             odom::RobotPosition lookahead) {
  // calculate whether the robot is on the left or right side of the circle
  float side = utils::sgn(std::sin(heading) * (lookahead.x - pose.x) -
                          std::cos(heading) * (lookahead.y - pose.y));
  // calculate center point and radius
  float a = -std::tan(heading);
  float c = std::tan(heading) * pose.x - pose.y;
  float x =
      std::fabs(a * lookahead.x + lookahead.y + c) / std::sqrt((a * a) + 1);
  float d = std::hypot(lookahead.x - pose.x, lookahead.y - pose.y);

  // return curvature
  return side * ((2 * x) / (d * d));
}

/**
 * @brief Move the chassis along a path
 *
 * @param path the path asset to follow
 * @param lookahead the lookahead distance. Units in inches. Larger values will
 * make the robot move faster but will follow the path less accurately
 * @param timeout the maximum time the robot can spend moving
 * @param forwards whether the robot should follow the path going forwards. true
 * by default
 * @param async whether the function should be run asynchronously. true by
 * default
 */
void odom::follow(const std::string& path, float lookahead, int timeout,
                  bool forwards, bool async) {
  // take the mutex
  mutex.take(TIMEOUT_MAX);
  // if the function is async, run it in a new task
  if (async) {
    pros::Task task(
        [&]() { follow(path, lookahead, timeout, forwards, false); });
    mutex.give();
    pros::delay(10);  // delay to give the task time to start
    return;
  }

  std::vector<odom::RobotPosition> pathPoints =
      getData(path);  // get list of path points
  odom::RobotPosition pose = getPosition();
  odom::RobotPosition lastPose = pose;
  odom::RobotPosition lookaheadPose(0, 0, 0);
  odom::RobotPosition lastLookahead = pathPoints.at(0);
  lastLookahead.theta = 0;
  float curvature;
  float targetVel;
  float prevLeftVel = 0;
  float prevRightVel = 0;
  int closestPoint;
  float leftInput = 0;
  float rightInput = 0;
  int compState = pros::competition::get_status();
  distTravelled = 0;

  // loop until the robot is within the end tolerance
  for (int i = 0;
       i < timeout / 10 && pros::competition::get_status() == compState; i++) {
    // get the current position of the robot
    pose = getPosition();
    if (!forwards) pose.theta -= M_PI;

    // update completion vars
    distTravelled += pose.distance(lastPose);
    lastPose = pose;

    // find the closest point on the path to the robot
    closestPoint = findClosest(pose, pathPoints);
    // if the robot is at the end of the path, then stop
    if (pathPoints.at(closestPoint).theta == 0) break;

    // find the lookahead point
    lookaheadPose = lookaheadPoint(lastLookahead, pose, pathPoints, lookahead);
    lastLookahead = lookaheadPose;  // update last lookahead position

    // get the curvature of the arc between the robot and the lookahead point
    float curvatureHeading = M_PI / 2 - pose.theta;
    curvature = findLookaheadCurvature(pose, curvatureHeading, lookaheadPose);

    // get the target velocity of the robot
    targetVel = pathPoints.at(closestPoint).theta;

    // calculate target left and right velocities
    float targetLeftVel =
        targetVel * (2 + curvature * drivetrain.trackWidth) / 2;
    float targetRightVel =
        targetVel * (2 - curvature * drivetrain.trackWidth) / 2;

    // ratio the speeds to respect the max speed
    float ratio =
        std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / 127;
    if (ratio > 1) {
      targetLeftVel /= ratio;
      targetRightVel /= ratio;
    }

    // update previous velocities
    prevLeftVel = targetLeftVel;
    prevRightVel = targetRightVel;

    // move the drivetrain
    if (forwards) {
      drivetrain.leftMotors->move(targetLeftVel);
      drivetrain.rightMotors->move(targetRightVel);
    } else {
      drivetrain.leftMotors->move(-targetRightVel);
      drivetrain.rightMotors->move(-targetLeftVel);
    }

    pros::delay(10);
  }

  // stop the robot
  drivetrain.leftMotors->move(0);
  drivetrain.rightMotors->move(0);
  // set distTravelled to -1 to indicate that the function has finished
  distTravelled = -1;
  // give the mutex back
  mutex.give();
}