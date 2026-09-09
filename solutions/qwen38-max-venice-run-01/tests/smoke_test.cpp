#include <vwmini/geometry.hpp>

#include <gtest/gtest.h>

namespace {

TEST(Smoke, Vec2ExactEquality)
{
    const vwmini::Vec2 a{1.0f, 2.0f};
    const vwmini::Vec2 b{1.0f, 2.0f};
    const vwmini::Vec2 c{1.0f, 2.5f};
    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
}

TEST(Smoke, AdditionIsComponentWise)
{
    const vwmini::Vec2 a{1.0f, 2.0f};
    const vwmini::Vec2 b{3.0f, 4.0f};
    const vwmini::Vec2 sum{4.0f, 6.0f};
    EXPECT_EQ(a + b, sum);
}

} // namespace
