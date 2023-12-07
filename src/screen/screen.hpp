#pragma once
#include "gif-pros/gifclass.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/widgets/lv_img.h"
#include "main.h"
namespace screen {

extern lv_obj_t *auton_selector_screen;

void initAutonSelector(Gif *gif);

}  // namespace screen