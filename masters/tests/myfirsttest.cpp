//
// Created by mirko on 06.01.24.
//
#include <gtest/gtest.h>
// Add more test cases as needed
// Demonstrate some basic assertions.
TEST(HelloTest, BasicAssertions) {
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);
}