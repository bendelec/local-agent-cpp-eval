#include <vwmini/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{

using vwmini::cross;
using vwmini::dot;
using vwmini::length;
using vwmini::normalized;
using vwmini::Vec2;

TEST(Vec2Test, EqualityIsExactComponentWise)
{
    EXPECT_EQ((Vec2{1.0f, 2.0f}), (Vec2{1.0f, 2.0f}));
    EXPECT_NE((Vec2{1.0f, 2.0f}), (Vec2{1.0f, 2.0f + 1e-5f}));
    EXPECT_NE((Vec2{0.0f, 0.0f}), (Vec2{std::numeric_limits<float>::denorm_min(), 0.0f}));
    EXPECT_EQ((Vec2{0.0f, 0.0f}), (Vec2{-0.0f, -0.0f})); // -0.0 == 0.0 in IEEE comparison
}

TEST(Vec2Test, ArithmeticOperators)
{
    const Vec2 a{1.0f, 2.0f};
    const Vec2 b{3.0f, 4.0f};

    EXPECT_EQ((a + b), (Vec2{4.0f, 6.0f}));
    EXPECT_EQ((a - b), (Vec2{-2.0f, -2.0f}));
    EXPECT_EQ((a * 2.0f), (Vec2{2.0f, 4.0f}));
    EXPECT_EQ((2.0f * a), (Vec2{2.0f, 4.0f}));
}

TEST(Vec2Test, DotAndCross)
{
    EXPECT_FLOAT_EQ(vwmini::dot(Vec2{1.0f, 2.0f}, Vec2{3.0f, 4.0f}), 11.0f);
    EXPECT_FLOAT_EQ(vwmini::cross(Vec2{1.0f, 0.0f}, Vec2{0.0f, 1.0f}), 1.0f);
    EXPECT_FLOAT_EQ(vwmini::cross(Vec2{0.0f, 1.0f}, Vec2{1.0f, 0.0f}), -1.0f);
}

TEST(Vec2Test, Length)
{
    EXPECT_FLOAT_EQ(length(Vec2{0.0f, 0.0f}), 0.0f);
    EXPECT_FLOAT_EQ(length(Vec2{3.0f, 4.0f}), 5.0f);
    EXPECT_FLOAT_EQ(length(Vec2{-3.0f, -4.0f}), 5.0f);
}

TEST(Vec2Test, LengthStaysFiniteForExtremeInputs)
{
    const float big = std::numeric_limits<float>::max() * 0.4f;
    EXPECT_TRUE(std::isfinite(length(Vec2{big, 0.0f})));
    EXPECT_LT(length(Vec2{1.0f, std::numeric_limits<float>::max()}),
              std::numeric_limits<float>::infinity());
    EXPECT_TRUE(std::isfinite(length(Vec2{1e-30f, 1e-30f})));
}

TEST(Vec2Test, NormalizedUnitVectors)
{
    const Vec2 unit_x = normalized(Vec2{3.0f, 0.0f});
    EXPECT_FLOAT_EQ(unit_x.x, 1.0f);
    EXPECT_FLOAT_EQ(unit_x.y, 0.0f);

    const Vec2 diagonal = normalized(Vec2{-2.0f, -2.0f});
    EXPECT_NEAR(diagonal.x, -0.70710678f, 1e-6f);
    EXPECT_NEAR(diagonal.y, -0.70710678f, 1e-6f);
    EXPECT_NEAR(length(normalized(Vec2{-1.5f, 2.5f})), 1.0f, 1e-6f);
}

TEST(Vec2Test, NormalizedZeroVectorIsZero)
{
    EXPECT_EQ(normalized(Vec2{0.0f, 0.0f}), (Vec2{0.0f, 0.0f}));
    EXPECT_EQ(normalized(Vec2{-0.0f, 0.0f}), (Vec2{0.0f, 0.0f}));
}

} // namespace
