#pragma once
#include "liblvgl/core/lv_obj.h"
#include "main.h"

namespace screen {

/**
 * Class that represents a window on the screen.
 */
class Window {
public:
  Window(int width, int height);
  Window(int width, int height, lv_obj_t *parent);
  ~Window();

  // delete the copy constructor and assignment operator
  Window(const Window &) = delete;
  Window &operator=(const Window &) = delete;

  /**
   * Gets the width of the window.
   *
   * @return The width of the window.
   */
  int getWidth() const;

  /**
   * Gets the height of the window.
   *
   * @return The height of the window.
   */
  int getHeight() const;

  /**
   * Gets the LVGL object that represents this window.
   *
   * @return The LVGL object that represents this window.
   */
  lv_obj_t *getLvObj();

  /**
   * Gets if this window is active
   */
  bool isActive() const;

  /**
   * Sets if this window is active
   */
  void setActive(bool active);

  /**
   * Toggles the active state of this window
   * @return the new active state
   */
  bool toggleActive();

  /**
   * Gets the pros task handle for this window.
   * nullptr if the task is not running.
   */
  pros::Task *getTask() const;

  /**
   * Starts the task for this window.
   */
  void startTask();

  /**
   * Initializes the window.
   */
  virtual void init();

protected:
  /**
   * The width of the window.
   */
  int width;

  /**
   * The height of the window.
   */
  int height;

  /**
   * The LVGL object that represents this window.
   */
  lv_obj_t *lvObj;

  /**
   * The pros task handle for this window.
   */
  pros::Task *task;

  /**
   * If this window is active
   */
  bool active;

  /**
   * The task function for this window.
   */
  virtual void tick();
};

/**
 * Field window class.
 */
class FieldWindow : public Window {
public:
  using Window::Window;

  // overridden functions
  void init() override;

protected:
  // overridden functions
  void tick() override;

private:
  lv_obj_t *fieldImage;
  lv_obj_t *robotImage;
};

} // namespace screen