#include "../odom/odom.hpp"
#include "./screen.hpp"
#include "main.h"

// current screen
lv_obj_t *original_screen = nullptr;

// create the second screen
lv_obj_t *screen::auton_selector_screen = lv_obj_create(NULL);

// handle switching between screens
static void switch_screen_event_cb(lv_event_t *event) {
  lv_event_code_t code = lv_event_get_code(event);
  lv_obj_t *obj = lv_event_get_target(event);

  Gif *gif = (Gif *)lv_event_get_user_data(event);

  if (code == LV_EVENT_CLICKED) {
    if (lv_scr_act() == screen::auton_selector_screen) {
      if (gif != NULL) gif->resume();
      lv_scr_load(original_screen);
    } else {
      if (gif != NULL) gif->pause();
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
    // if (value == 0) {
    //   odom::autonomous = odom::Autonomous::ScoreLeft;
    // } else if (value == 1) {
    //   odom::autonomous = odom::Autonomous::ScoreSimple;
    // } else if (value == 2) {
    //   odom::autonomous = odom::Autonomous::Skills;
    // } else {
    //   odom::autonomous = odom::Autonomous::None;
    // }
  }
}

void screen::initAutonSelector(Gif *gif) {
  // get current active screen
  original_screen = lv_scr_act();

  // register the gif event callback
  lv_obj_t *autoselect = lv_btn_create(original_screen);
  lv_obj_set_pos(autoselect, lv_obj_get_width(lv_scr_act()) - 80, 10);
  lv_label_set_text(lv_label_create(autoselect), "Menu");
  lv_obj_add_event_cb(autoselect, switch_screen_event_cb, LV_EVENT_CLICKED,
                      gif);

  // create the buttons for the auton selector
  lv_obj_t *back = lv_btn_create(screen::auton_selector_screen);
  lv_obj_t *back_label = lv_label_create(back);

  // set the text of the back button
  lv_label_set_text(back_label, "Back");
  lv_obj_add_event_cb(back, switch_screen_event_cb, LV_EVENT_CLICKED, gif);

  // set the position of the back button
  lv_obj_set_pos(back, lv_obj_get_width(lv_scr_act()) - 80, 10);

  // create dropdown for auton selector
  lv_obj_t *dropdown = lv_dropdown_create(screen::auton_selector_screen);
  lv_obj_set_pos(dropdown, 10, 10);

  // create the options for the dropdown
  lv_dropdown_set_options(dropdown,
                          "Score Left\nScore Simple\nTouch Bar\nSkills\nNone");
  lv_obj_add_event_cb(dropdown, auto_dropdown_select, LV_EVENT_VALUE_CHANGED,
                      NULL);
}