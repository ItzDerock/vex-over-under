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

  if (code == LV_EVENT_CLICKED) {
    if (lv_scr_act() == screen::auton_selector_screen)
      lv_scr_load(original_screen);
    else
      lv_scr_load(screen::auton_selector_screen);
  }
}

// handle changes to the auton selector screen
static void auto_dropdown_select(lv_event_t *event) {
  lv_event_code_t code = lv_event_get_code(event);
  lv_obj_t *obj = lv_event_get_target(event);

  if (code == LV_EVENT_VALUE_CHANGED) {
    // std::string value = lv_dropdown_get_selected_str(obj);
    int value = lv_dropdown_get_selected(obj);
    if (value == 0) {
      odom::autonomous = odom::Autonomous::Red;
    } else if (value == 1) {
      odom::autonomous = odom::Autonomous::Blue;
    } else if (value == 2) {
      odom::autonomous = odom::Autonomous::Skills;
    } else {
      odom::autonomous = odom::Autonomous::None;
    }
  }
}

void screen::initAutonSelector(Gif *gif) {
  // get current active screen
  original_screen = lv_scr_act();

  // register the gif event callback
  lv_obj_add_event_cb(gif->getCanvas(), switch_screen_event_cb,
                      LV_EVENT_CLICKED, NULL);

  // create the buttons for the auton selector
  lv_obj_t *back = lv_btn_create(screen::auton_selector_screen);
  lv_obj_t *back_label = lv_label_create(back);

  // set the text of the back button
  lv_label_set_text(back_label, "Back");
  lv_obj_add_event_cb(back, switch_screen_event_cb, LV_EVENT_CLICKED, NULL);

  // set the position of the back button
  lv_obj_set_pos(back, 0, 0);
  lv_obj_set_size(back, 100, 50);

  // create dropdown for auton selector
  lv_obj_t *dropdown = lv_dropdown_create(screen::auton_selector_screen);
  lv_obj_set_pos(dropdown, 0, 50);
  lv_obj_set_size(dropdown, 100, 50);

  // create the options for the dropdown
  lv_dropdown_set_options(dropdown, "Red\nBlue\nSkills\nNone");
  lv_obj_add_event_cb(dropdown, auto_dropdown_select, LV_EVENT_VALUE_CHANGED,
                      NULL);

  // create the label for the dropdown
  lv_obj_t *dropdown_label = lv_label_create(dropdown);
  lv_label_set_text(dropdown_label, "Autonomous");
}