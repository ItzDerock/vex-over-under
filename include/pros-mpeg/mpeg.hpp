#pragma once
#include "main.h"

#define PL_MPEG_IMPLEMENTATION
#include "pros-mpeg/pl_mpeg.h"

class MPEGPlayer {
 public:
  MPEGPlayer(const char* fname, lv_obj_t* parent);
  ~MPEGPlayer();

 private:
  pros::Task* task;
  plm_t* decoder;
  int stride;

  // raw rgb frame data
  uint8_t* frame;

  // lvgl frame data
  lv_color_t* lvframe;
  lv_obj_t* canvas;

  int width;
  int height;
  double frametime;

  void _render();

  static void _render_task(void* arg);
  static void _decode_callback(plm_t* self, plm_frame_t* frame, void* user);
};