#include "display/lv_core/lv_obj.h"
#include "main.h"
#include "screen.hpp"

using namespace screen;

// refresh rate of the screen in milliseconds
const int SCREEN_REFRESH_RATE = 20;

/**
 * Main constructor for a window.
 */
Window::Window(int width, int height, lv_obj_t *parent) {
  this->width = width;
  this->height = height;
  this->active = true;
  this->lvObj = lv_obj_create(parent, NULL);
  lv_obj_set_size(this->lvObj, width, height);
}

/**
 * Creates a new window with the given width and height and uses the active
 * screen as the parent.
 */
Window::Window(int width, int height) : Window(width, height, lv_scr_act()) {}

/**
 * Frees the LVGL object that represents this window and kills the pros task.
 */
Window::~Window() {
  lv_obj_del(this->lvObj);

  if (this->task != nullptr) {
    this->task->remove();
    delete this->task;
  }
}

void Window::startTask() {
  if (this->task != nullptr) {
    // error - task already started
    throw "Task already started";
  }

  this->task = new pros::Task(
      [this]() {
        while (true) {
          this->tick();
          pros::delay(SCREEN_REFRESH_RATE);
        }
      },
      "Screen Window");

  // this->task->set_priority(SCREEN_TASK_PRIORITY);
}

/**
 * Gets the width of the window.
 *
 * @return The width of the window.
 */
int Window::getWidth() { return this->width; }

/**
 * Gets the height of the window.
 *
 * @return The height of the window.
 */
int Window::getHeight() { return this->height; }

/**
 * Gets the LVGL object that represents this window.
 *
 * @return The LVGL object that represents this window.
 */
lv_obj_t *Window::getLvObj() { return this->lvObj; }

/**
 * Gets if this window is active
 */
bool Window::isActive() { return this->active; }

/**
 * Sets if this window is active
 */
void Window::setActive(bool active) { this->active = active; }

/**
 * Toggles the active state of this window
 * @return the new active state
 */
bool Window::toggleActive() {
  this->active = !this->active;
  return this->active;
}

/**
 * Gets the pros task handle for this window.
 * nullptr if the task is not running.
 */
pros::Task *Window::getTask() { return this->task; }

// stub
void Window::tick(){};