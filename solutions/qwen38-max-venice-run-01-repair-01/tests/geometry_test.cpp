#include <vwmini/geometry.hpp>

#include "test_util.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace {

using vwmini::Vec2;
using vwmini::test::V;

constexpr float kFloatMax = std::numeric_limits<float>::max();

TEST(Geometry, ExactEqualityIsComponentWise)
{
    EXPECT_EQ(V(1.0f, 2.0f), V(1.0f, 2.0f));
    // nextafter yields the next representable float, guaranteeing exact inequality.
    EXPECT_NE(V(1.0f, 2.0f), V(1.0f, std::nextafter(2.0f, 3.0f)));
}

TEST(Geometry, ArithmeticOperators)
{
    EXPECT_EQ(V(1.0f, 2.0f) + V(3.0f, 4.0f), V(4.0f, 6.0f));
    EXPECT_EQ(V(3.0f, 4.0f) - V(1.0f, 2.0f), V(2.0f, 2.0f));
    EXPECT_EQ(V(1.0f, 2.0f) * 3.0f, V(3.0f, 6.0f));
    EXPECT_EQ(3.0f * V(1.0f, 2.0f), V(3.0f, 6.0f));
}

TEST(Geometry, DotAndCross)
{
    EXPECT_FLOAT_EQ(vwmini::dot(V(1.0f, 2.0f), V(3.0f, 4.0f)), 11.0f);
    EXPECT_FLOAT_EQ(vwmini::cross(V(1.0f, 0.0f), V(0.0f, 1.0f)), 1.0f);
    EXPECT_FLOAT_EQ(vwmini::cross(V(0.0f, 1.0f), V(1.0f, 0.0f)), -1.0f);
}

TEST(Geometry, LengthIsEuclideanAndFinite)
{
    EXPECT_FLOAT_EQ(vwmini::length(V(3.0f, 4.0f)), 5.0f);
    EXPECT_FLOAT_EQ(vwmini::length(V(0.0f, 0.0f)), 0.0f);
}

TEST(Geometry, NormalizedProducesUnitVector)
{
    const Vec2 n = vwmini::normalized(V(3.0f, 4.0f));
    EXPECT_FLOAT_EQ(n.x, 0.6f);
    EXPECT_FLOAT_EQ(n.y, 0.8f);
    EXPECT_FLOAT_EQ(vwmini::length(n), 1.0f);
}

TEST(Geometry, NormalizedZeroReturnsZero)
{
    EXPECT_EQ(vwmini::normalized(V(0.0f, 0.0f)), V(0.0f, 0.0f));
}

TEST(Geometry, LengthSaturatesWhenTrueMagnitudeExceedsFloatRange)
{
    // hypot(FLT_MAX, FLT_MAX) ~= 4.8e38 overflows float; the contract demands
    // finite output for finite input, so the result saturates at FLT_MAX.
    EXPECT_TRUE(std::isfinite(vwmini::length(V(kFloatMax, kFloatMax))));
    EXPECT_FLOAT_EQ(vwmini::length(V(kFloatMax, kFloatMax)), kFloatMax);
    EXPECT_FLOAT_EQ(vwmini::length(V(kFloatMax, 0.0f)), kFloatMax);
    EXPECT_FLOAT_EQ(vwmini::length(V(-kFloatMax, -kFloatMax)), kFloatMax);
}

TEST(Geometry, NormalizedHandlesVectorsWhoseFloatLengthOverflows)
{
    // The float-only computation divided by an infinite length and collapsed
    // such vectors to {0, 0}; double division keeps the unit-vector contract.
    const Vec2 axis = vwmini::normalized(V(kFloatMax, 0.0f));
    EXPECT_FLOAT_EQ(axis.x, 1.0f);
    EXPECT_FLOAT_EQ(axis.y, 0.0f);
    const Vec2 diagonal = vwmini::normalized(V(kFloatMax, kFloatMax));
    EXPECT_NEAR(vwmini::length(diagonal), 1.0f, 1e-5f);
    EXPECT_FLOAT_EQ(diagonal.x, diagonal.y);
}

} // namespace
