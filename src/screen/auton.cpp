#include "../config.hpp"
#include "main.h"
#include "pros-mpeg/mpeg.hpp"
#include "robot/odom.hpp"
#include "robot/screen.hpp"
#include "robot/utils.hpp"

// current screen
lv_obj_t *original_screen = nullptr;

// create the second screen
lv_obj_t *screen::auton_selector_screen = lv_obj_create(NULL);

// handle switching between screens
static void switch_screen_event_cb(lv_event_t *event) {
  lv_event_code_t code = lv_event_get_code(event);
  lv_obj_t *obj = lv_event_get_target(event);

  MPEGPlayer *player = (MPEGPlayer *)lv_event_get_user_data(event);

  if (code == LV_EVENT_CLICKED) {
    if (lv_scr_act() == screen::auton_selector_screen) {
      if (player != NULL) player->resume();
      lv_scr_load(original_screen);
    } else {
      if (player != NULL) player->pause();
      lv_scr_load(screen::auton_selector_screen);
    }
  }
}

// handle changes to the auton selector screen
static void auto_dropdown_select(lv_event_t *event) {
  lv_event_code_t code = lv_event_get_code(event);
  lv_obj_t *obj = lv_event_get_target(event);

  if (code == LV_EVENT_VALUE_CHANGED) {
    // std::string value = lv_dropdown_get_selected_str(obj);
    int value = lv_dropdown_get_selected(obj);
    odom::autonomous = (odom::Autonomous)value;
  }
}

static void reset_position_event_cb(lv_event_t *event) {
  lv_event_code_t code = lv_event_get_code(event);
  // lv_obj_t *obj = lv_event_get_target(event);

  if (code == LV_EVENT_CLICKED) {
    switch (odom::autonomous) {
      case odom::Autonomous::Skills:
        odom::reset(
            {-35, -70 + (double)DRIVE_TRACK_WIDTH / 2, utils::degToRad(90)});
        break;

      case odom::Autonomous::SixBall:
        odom::reset(
            {-8, -70 + (double)DRIVE_TRACK_WIDTH / 2, utils::degToRad(90)});
        break;

      case odom::Autonomous::Defense:
        odom::reset({-60, -40, M_PI});
        break;

      default:
        odom::reset({0, 0, 0});
        break;
    }
  }
}

void screen::initAutonSelector(MPEGPlayer *video) {
  // get current active screen
  original_screen = lv_scr_act();

  // register the gif event callback
  lv_obj_t *autoselect = lv_btn_create(original_screen);
  lv_obj_set_pos(autoselect, lv_obj_get_width(lv_scr_act()) - 80, 10);
  lv_label_set_text(lv_label_create(autoselect), "Menu");
  lv_obj_add_event_cb(autoselect, switch_screen_event_cb, LV_EVENT_CLICKED,
                      video);

  // create the buttons for the auton selector
  lv_obj_t *back = lv_btn_create(screen::auton_selector_screen);
  lv_obj_t *back_label = lv_label_create(back);

  // set the text of the back button
  lv_label_set_text(back_label, "Back");
  lv_obj_add_event_cb(back, switch_screen_event_cb, LV_EVENT_CLICKED, video);

  // set the position of the back button
  lv_obj_set_pos(back, lv_obj_get_width(lv_scr_act()) - 80, 10);

  // create dropdown for auton selector
  lv_obj_t *dropdown = lv_dropdown_create(screen::auton_selector_screen);
  lv_obj_set_pos(dropdown, 10, 10);
  lv_dropdown_set_selected(dropdown, (uint16_t)odom::autonomous);

  // create the options for the dropdown
  lv_dropdown_set_options(dropdown,
                          "None\nSkills\nSix Ball\nTouch Bar\nDefense");
  lv_obj_add_event_cb(dropdown, auto_dropdown_select, LV_EVENT_VALUE_CHANGED,
                      NULL);

  // reset position button
  lv_obj_t *reset_position = lv_btn_create(screen::auton_selector_screen);
  lv_obj_t *reset_position_label = lv_label_create(reset_position);

  // set the text of the reset position button
  lv_label_set_text(reset_position_label, "Reset Position");
  lv_obj_add_event_cb(reset_position, reset_position_event_cb, LV_EVENT_CLICKED,
                      NULL);

  // set the position of the reset position button
  lv_obj_set_pos(reset_position, 10, 50);

  // text that shows current odom pose
  lv_obj_t *odom_text = lv_label_create(screen::auton_selector_screen);
  lv_label_set_text(odom_text, "odom");
  lv_obj_set_pos(odom_text, 10, 90);

  pros::Task([odom_text]() {
    while (true) {
      odom::RobotPosition position = odom::getPosition(true);

      std::string text = "x: " + std::to_string(position.x) +
                         "\ny: " + std::to_string(position.y) +
                         "\ntheta: " + std::to_string(position.theta) +
                         "\nvelocity: " + std::to_string(odom::getVelocity()) +
                         " in/sec";

      lv_label_set_text(odom_text, text.c_str());
      pros::delay(50);
    }
  });
}