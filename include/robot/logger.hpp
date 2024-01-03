#pragma once
#include <vector>

/**
 * Simple telemetry logger
 */

namespace logger {

/**
 * Corresponds to an MQTT topic
 */
enum class Route {

  // Expects x,y,theta
  RobotPosition,

  // Expects left,right
  RobotVelocity,

  //// PURE PURSUIT /////////////
  // Expects x,y,theta
  PurePursuitLookaheadPose,

  // Expects <number>
  PurePursuitCurvature,

  // Expects <number>
  PurePursuitTargetVelocity
};

/**
 * Logs to stdout
 * STDOUT FORMAT: {route}, {data1}, {data2}, ...
 *
 * @param route The route to log to
 * @param data the data to log
 */
void log(Route route, std::vector<double> const& data);

}  // namespace logger