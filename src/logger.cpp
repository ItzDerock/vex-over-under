#include "robot/logger.hpp"

#include <stdarg.h>
#include <stdio.h>

#include "main.h"

void logger::log(Route route, std::vector<double> const& data) {
  printf("TELE_DEBUG:%d", (int)route);
  for (auto const& d : data) {
    printf(",%f", d);
  }
  printf("TELE_END\n");
}