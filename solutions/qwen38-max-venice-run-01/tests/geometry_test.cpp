#include <vwmini/geometry.hpp>

#include "test_util.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using vwmini::Vec2;
using vwmini::test::V;

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

} // namespace
