#include "gif-pros/gifclass.hpp"

/**
 * MIT License
 * Copyright (c) 2019 Theo Lemay
 * https://github.com/theol0403/gif-pros
 *
 * Heavily modified to support a newer version of gifdec.h
 * by Derock X. (derock@derock.dev)
 * https://github.com/ItzDerock/gif-pros
 */

/**
 * Construct the Gif class
 * An abstraction over gifdec.h
 *
 * @param fname  the gif filename on the SD card (prefixed with /usd/)
 * @param parent the LVGL parent object
 */
Gif::Gif(const char *fname, lv_obj_t *parent) {
  // 1. Open the gif file
  _gif = gd_open_gif(fname);

  if (_gif == NULL) {
    std::cerr << "Gif::Gif - error opening \"" + std::string(fname) + "\""
              << std::endl;
    return;
  }

  // 1.1 Optional: print out gif info
  std::cout << "Gif::Gif - \"" << fname << "\" info:" << std::endl;
  std::cout << "  width: " << _gif->width << std::endl;
  std::cout << "  height: " << _gif->height << std::endl;
  std::cout << "  colors: " << _gif->palette->size << std::endl;

  // 2. Allocate memory for current frame
  _buffer = new uint8_t[_gif->width * _gif->height * 3];
  _cbuf = new lv_color_t[_gif->width * _gif->height];
  _canvas = lv_canvas_create(parent);
  lv_canvas_set_buffer(_canvas, _cbuf, _gif->width, _gif->height,
                       LV_IMG_CF_TRUE_COLOR_ALPHA);

  // 3. Create task to render gif
  _task = pros::c::task_create(
      _render_task, this, TASK_PRIORITY_DEFAULT - 2, TASK_STACK_DEPTH_DEFAULT,
      ("GIF - \"" + std::string(fname) + "\"").c_str());
};

/**
 * Destructs and cleans the Gif class
 */
Gif::~Gif() { _cleanup(); }

/**
 * Pauses the GIF task
 */
void Gif::pause() { pros::c::task_suspend(_task); }

/**
 * Resumes the GIF task
 */
void Gif::resume() { pros::c::task_resume(_task); }

/**
 * Deletes GIF and frees all allocated memory
 */
void Gif::clean() { _cleanup(); }

/**
 * Returns the LVGL object pointer
 */
lv_obj_t *Gif::getCanvas() const { return _canvas; }

/**
 * Cleans and frees all allocated memory
 */
void Gif::_cleanup() {
  if (_canvas) {
    lv_obj_del(_canvas);
    _canvas = nullptr;
  }
  if (_cbuf) {
    delete[] _cbuf;
    _cbuf = nullptr;
  }
  if (_buffer) {
    free(_buffer);
    _buffer = nullptr;
  }
  if (_gif) {
    gd_close_gif(_gif);
    _gif = nullptr;
  }
  // deleting task kills this thread
  if (_task) {
    pros::c::task_delete(_task);
    _task = nullptr;
  }
}

/**
 * Render cycle, blocks until loop count exceeds gif loop count flag (if any)
 */
void Gif::_render() {
  for (size_t looped = 1;; looped++) {
    while (gd_get_frame(_gif)) {
      auto now = pros::millis();

      gd_render_frame(_gif, _buffer);

      auto renderdone = pros::millis();
      if (renderdone - now > 10) {
        std::cerr << "Gif::_render - frame render took longer than 10ms. Took "
                  << pros::millis() - now << "ms" << std::endl;
      }

      uint8_t *color = _buffer;  // uint8_t[3] pointer
      for (size_t i = 0; i < _gif->height * _gif->width; i++) {
        // check for transparency
        // in gifs, there are no partial transparency, only fully
        // transparent or fully opaque
        _cbuf[i].ch.alpha =
            gd_is_bgcolor(_gif, color) ? LV_OPA_TRANSP : LV_OPA_COVER;

        // break the 24-bit color into 8-bit channels
        // deref then increment pointer
        _cbuf[i].ch.red = *color++;
        _cbuf[i].ch.green = *color++;
        _cbuf[i].ch.blue = *color++;
      };

      if (pros::millis() - renderdone > 10) {
        std::cerr << "Gif::_render - frame conversion took longer than 10ms. "
                  << "Took " << pros::millis() - renderdone << "ms"
                  << std::endl;
      }

      lv_obj_invalidate(_canvas);  // force canvas redraw

      auto delay = _gif->gce.delay * 10;
      auto delta = pros::millis() - now;
      delay -= delta;

      // std::cerr << "Gif::_render - frame render took " << delta << "ms"
      //           << "delay is " << delay << "ms" << std::endl;

      pros::delay(delay > 0 ? delay : 10);
      // if (delay > 0) pros::delay(delay);
    }

    if (looped == _gif->loop_count) {
      break;
    }

    gd_rewind(_gif);
  }

  _cleanup();
}

/**
 * Calls _render()
 * @param arg Gif*
 */
void Gif::_render_task(void *arg) {
  Gif *instance = static_cast<Gif *>(arg);
  instance->_render();
}