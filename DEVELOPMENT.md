# Pure Pursuit

By setting `PURE_PURSUIT_DEBUG` to `true` in `pursuit.cpp`, you can enable debug mode. This will send a bunch of data over the serial port to the computer. This data can be used to plot the path, the robot, and the lookahead point.

## Keys
- `targetLeftVel` and `targetRightVel` 
  - Format: `<float>`
  - The **adjusted** left and right wheel velocities.
  - (-127, 127) as a float. 
  - Ratio'd down to the max velocity.
  - Adjusted to account for backwards driving.
- `pose`
  - Format: `<float>, <float>, <float>` 
  - `x`, `y`, `theta` of the robot (radians)
- `targetPose`
  - Format: `<float>, <float>, <float>` 
  - `x`, `y`, `theta` of the target (radians)
- `pathPoints.size()`
  - Format: `<int>`
  - The number of points in the path.
  - Sent at the beginning of the path.
- `lookaheadPoint`
  - Format: `<float>, <float>, <float>`
  - `x`, `y`, `theta` of the lookahead point (radians)
- `curvature`
  - Format: `<float>`
  - The curvature of the path at the lookahead point.
- `targetVel`
  - Format: `<float>`
  - The target velocity of the robot at the lookahead point.
- `actualLeftVel` and `actualRightVel`
  - Format: `<float>`
  - The actual left and right wheel velocities in RPM as reported by the encoders.

## Format
Very simple, just a key and a value separated by a colon and a space: `key: value\n`

## Example
```
pathPoints.size(): 3
targetLeftVel: 0.000000
targetRightVel: 0.000000
pose: 0.000000, 0.000000, 0.000000
targetPose: 0.000000, 0.000000, 0.000000
```