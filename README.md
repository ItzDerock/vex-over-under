# VEX 2023 - Over Under Code
This repository holds the C++ code for KCD's VEX 2023 Over Under robot (95993A).

## Features:
- Simple tank drive controls
- 3 wheel odometry system
  - uses 2 internal encoders
  - 1 exterior encoder for lateral movement
- PID Control
- Catapult control
  - Rotational sensor to read position
  - Single press to fire once
  - Hold for rapid fire
- Wings toggle
- GIF decoder
  - Plays a gif on the brain screen using LVGL and a simple gifdecoder library

## Ports:
See [src/config.hpp] for the port configuration. Can be easily edited to accomodate broken ports.

## Technology Used:
PROSv5 - C++ API to interface with the VEX hardware
LVGL - C library for graphics rendering on the brain screen
gifdec - C library for decoding gifs

## Utility Programs:
- `scripts/htmlify.sh` - Converts all source files (controlled by a list containing glob patterns) to one large HTML file so you can export and paste into your design notebook.