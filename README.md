# VEX 2023 - Over Under Code
This repository holds the C++ code for KCD's VEX 2023 Over Under robot (95993A).

## __Post Competition Update__
This update was written after the Kentucky HS State Championship. Our 5 ball autonomous was highly successful, probably one of the best at the comp. We ended up 13th after qualis (6 wins, 2 losses, somehow being put after teams that had a 50% win ratio). We managed to get 7th in skills (tied for 3rd best prog score, 159 driver & 147 prog). 

Unfortunately we got DQ'd during first elimination round because a very tinnny part of our robot touched the opponent elevation bar as we were driving underneath it during last 15 secs.

## Features:
- Simple tank drive controls
- 3 wheel odometry system
  - uses 2 internal encoders
  - 1 exterior encoder for lateral movement 
- PID Control
- Boomerang Control
  - For making curved turns
- Pure Pursuit
  - Path tracking algorithm
- Catapult control
  - Rotational sensor to read position
  - Single press to fire once
  - Hold for rapid fire
- Wings toggle
- Video decoder
  - Plays a video on the brain screen using LVGL and a simple mpeg1 decoder library
  - Can play up to 40fps. Will automatically calculate the correct framerate for the video `/usd/game.mpeg`

## Ports:
See [[src/config.hpp]] for the port configuration. Can be easily edited to accomodate broken ports.

## Technology Used:
- [PROSv5](https://github.com/purduesigbots/pros) - C++ API to interface with the VEX hardware
- [LVGL](https://lvgl.io/) - C library for graphics rendering on the brain screen
- ~~[gifdec](https://github.com/lecram/gifdec) - C library for decoding gifs~~ GIFs have been replaced with:
- [pl_mpeg](https://github.com/phoboslab/pl_mpeg) - C library to decode MPEG1 video files.
- [GoogleTests](https://github.com/google/googletest) - C++ library for unit testing

## Testing:
> [!NOTE]  
> Tests are currently broken since the code is very coupled to the pros-library and it is not easy to run code meant for the v5's ARM CPU on an x86_64 machine.

To run the tests, run the following command:
```sh
# Generate configuration
cmake -S . -B build

# Build and run tests
cmake --build build
./build/Robot
```

## Utility Programs:
- `scripts/htmlify.sh` - Converts all source files (controlled by a list containing glob patterns) to one large HTML file so you can export and paste into your design notebook.
  - After using this, use `pdmtoppm -r 250 -png <pdf> page -progress` to convert to PNGs 
  - Then use LibreOffice Impress, import > media > image gallery to import images
  - Then export as .pptx and import that to Google Slides
  - Kind of a pain, but most HTML/PDF -> PPTX converters mess up the formatting.

## Converting videos:
Install [`ffmpeg`](https://ffmpeg.org/) and run
```sh
$ ffmpeg -i ./your-file.mp4 ./output.mpeg
```
