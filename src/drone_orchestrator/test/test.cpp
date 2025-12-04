#include <gtest/gtest.h>
#include "orchestrator.hpp"
#include "my_dummy_lib_funct2.hpp"

TEST(dummy_test, this_should_pass) {
  EXPECT_EQ(1, 1);
}

TEST(dummy_test, this_should_pass_too) {
  EXPECT_EQ(function2(3), function2(3));
}

