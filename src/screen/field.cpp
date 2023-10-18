#include "../odom/odom.hpp"
#include "main.h"
#include "screen.hpp"

// The width of the field in inches (for use in converting inches to pixels)
const double FIELD_WIDTH_INCHES = 147.8377757;

// the width/height of the robot indicator in pixels
const int ROBOT_INDICATOR_SIZE = 15;

// load the field and robot image
LV_IMG_DECLARE(FIELD_BACKGROUND);
LV_IMG_DECLARE(ROBOT_IMAGE);

/**
 * Converts inches to pixels.
 *
 * @param[in] inches The number of inches to convert.
 * @param[in] imageWidth The width of the image in pixels.
 * @return The number of pixels that corresponds to the given number of inches.
 */
inline double inchesToPixels(double inches, double imageWidth) {
  return inches / FIELD_WIDTH_INCHES * imageWidth;
}

void screen::FieldWindow::init() {
  // create the images
  fieldImage = lv_img_create(this->lvObj);
  lv_img_set_src(fieldImage, &FIELD_BACKGROUND);

  robotImage = lv_img_create(this->lvObj);
  lv_img_set_src(robotImage, &ROBOT_IMAGE);

  // set the size of the images
  lv_obj_set_size(fieldImage, width, height);
  lv_obj_set_size(robotImage, ROBOT_INDICATOR_SIZE, ROBOT_INDICATOR_SIZE);

  // set the position of the robot image
  lv_obj_set_pos(robotImage, 0, 0);
  lv_obj_set_pos(robotImage, 0, 0);
}

void screen::FieldWindow::tick() {
  // get the robot's position
  auto position = odom::getPosition();

  // convert the position to pixels
  double x = inchesToPixels(position.x, width);
  double y = inchesToPixels(position.y, height);

  // set the position of the robot image
  lv_obj_set_pos(robotImage, x - ROBOT_INDICATOR_SIZE / 2.0,
                 height - y - ROBOT_INDICATOR_SIZE / 2.0);
}