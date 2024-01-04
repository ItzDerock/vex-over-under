/**
 * MPEGPlayer
 * Ports MPEG1 support to LVGL-based displays such as the VEX v5 Brain.
 * Built by Derock <derock@derock.dev>
 * Please credit if used
 * MIT License
 */

#include "pros-mpeg/mpeg.hpp"

#include "main.h"
#include "pros-mpeg/pl_mpeg.h"

MPEGPlayer::MPEGPlayer(const char *fname, lv_obj_t *parent) {
  // initialize decoder
  decoder = plm_create_with_filename(fname);

  // only decode video since brain does not have audio
  plm_set_video_enabled(decoder, true);
  plm_set_audio_enabled(decoder, false);

  // decode callback
  plm_set_video_decode_callback(decoder, _decode_callback, this);

  // enable looping
  plm_set_loop(decoder, true);

  // parse basic data
  width = plm_get_width(decoder);
  height = plm_get_height(decoder);
  frametime = (double)1000 / plm_get_framerate(decoder);

  // create rgb buffer
  frame = new uint8_t[width * height * 3];
  lvframe = new lv_color_t[width * height];

  // create LVGL canvas
  canvas = lv_canvas_create(parent);
  lv_canvas_set_buffer(canvas, lvframe, width, height, LV_IMG_CF_TRUE_COLOR);

  // start an internal task
  task = new pros::Task(_render_task, (void *)this, TASK_PRIORITY_DEFAULT - 2,
                        TASK_STACK_DEPTH_DEFAULT,
                        ("MPEG - \"" + std::string(fname) + "\"").c_str());
}

lv_obj_t *MPEGPlayer::getLVGLObj() const { return canvas; }
void MPEGPlayer::pause() { task->suspend(); }
void MPEGPlayer::resume() { task->resume(); }

/**
 * Basically just free's everything
 */
MPEGPlayer::~MPEGPlayer() {
  if (canvas) {
    lv_obj_del(canvas);
    canvas = nullptr;
  }

  if (lvframe) {
    delete[] lvframe;
    lvframe = nullptr;
  }

  if (frame) {
    delete[] frame;
    frame = nullptr;
  }

  if (decoder) {
    plm_destroy(decoder);
    decoder = nullptr;
  }

  if (task) {
    task->remove();
    task = nullptr;
  }
}

/**
 * Render loop
 */
void MPEGPlayer::_render() {
  uint32_t time = pros::millis();
  do {
    uint32_t delta = pros::millis() - time;
    time = pros::millis();

    plm_frame_t *frame = plm_decode_video(decoder);
    this->_decode_callback(decoder, frame, this);

    pros::delay(frametime);
  } while (!plm_has_ended(decoder));
}

/**
 * Callback -- draw to LVGL
 */
void MPEGPlayer::_decode_callback(plm_t *self, plm_frame_t *frame, void *arg) {
  MPEGPlayer *instance = static_cast<MPEGPlayer *>(arg);
  plm_frame_to_rgb(frame, instance->frame, instance->width * 3);

  // now draw to LVGL
  uint8_t *color = instance->frame;  // uint8_t[3] pointer
  for (size_t i = 0; i < instance->height * instance->width; i++) {
    // break the 24-bit color into 8-bit channels
    // deref then increment pointer
    instance->lvframe[i].ch.red = *color++;
    instance->lvframe[i].ch.green = *color++;
    instance->lvframe[i].ch.blue = *color++;
  };

  // invalidate
  lv_obj_invalidate(instance->canvas);
}

/**
 * Calls _render()
 * @param arg Gif*
 */
void MPEGPlayer::_render_task(void *arg) {
  MPEGPlayer *instance = static_cast<MPEGPlayer *>(arg);
  instance->_render();
}