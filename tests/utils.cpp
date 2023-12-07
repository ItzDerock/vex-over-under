#include "robot/utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(UtilsTest, angleSquish) {
  EXPECT_EQ(utils::angleSquish(4 * M_PI), 0);
  EXPECT_EQ(utils::angleSquish(4 * M_PI + 2), 2);
  EXPECT_EQ(utils::angleSquish(-4 * M_PI), 0);
  EXPECT_EQ(utils::angleSquish(-4 * M_PI + 2), 2);
}

TEST(something, sometjhing) {}