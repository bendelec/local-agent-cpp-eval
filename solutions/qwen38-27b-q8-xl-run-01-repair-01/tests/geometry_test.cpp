#include <vwmini/geometry.hpp>

#include "vwmini/geometry_priv.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using vwmini::cross;
using vwmini::detail::is_finite;
using vwmini::detail::point_on_segment_exact;
using vwmini::detail::point_segment_distance;
using vwmini::dot;
using vwmini::length;
using vwmini::normalized;
using vwmini::Vec2;

namespace {

constexpr float kTol = 1e-5f;

} // namespace

// MSH-001
TEST(Geometry, Vec2Arithmetic)
{
    const Vec2 a{1.0f, 2.0f};
    const Vec2 b{3.0f, -4.0f};
    EXPECT_EQ(a + b, (Vec2{4.0f, -2.0f}));
    EXPECT_EQ(a - b, (Vec2{-2.0f, 6.0f}));
    EXPECT_EQ(a * 2.0f, (Vec2{2.0f, 4.0f}));
    EXPECT_EQ(2.0f * a, (Vec2{2.0f, 4.0f}));
    EXPECT_EQ(dot(a, b), -5.0f);
    EXPECT_EQ(cross(a, b), -10.0f);
    // Exact component-wise equality.
    EXPECT_TRUE((Vec2{0.5f, 1.0f} == Vec2{0.5f, 1.0f}));
    EXPECT_FALSE((Vec2{0.5f, 1.0f} == Vec2{0.5f, 1.0f + 1e-7f}));
}

TEST(Geometry, LengthBasics)
{
    EXPECT_FLOAT_EQ(length(Vec2{0.0f, 0.0f}), 0.0f);
    EXPECT_NEAR(length(Vec2{3.0f, 4.0f}), 5.0f, kTol);
    EXPECT_NEAR(length(Vec2{-3.0f, 4.0f}), 5.0f, kTol);
    EXPECT_GE(length(Vec2{1.0f, 1.0f}), 0.0f);
}

TEST(Geometry, Normalized)
{
    // Zero input must return {0, 0} (MSH-001).
    EXPECT_EQ(normalized(Vec2{0.0f, 0.0f}), (Vec2{0.0f, 0.0f}));
    const Vec2 u = normalized(Vec2{3.0f, 4.0f});
    EXPECT_NEAR(u.x, 0.6f, kTol);
    EXPECT_NEAR(u.y, 0.8f, kTol);
    EXPECT_NEAR(length(u), 1.0f, kTol);
    // Non-finite input stays finite and safe.
    const Vec2 safe = normalized(Vec2{std::numeric_limits<float>::infinity(), 1.0f});
    EXPECT_TRUE(std::isfinite(safe.x) && std::isfinite(safe.y));
}

// S1.1: Vec2 finiteness classification.
TEST(Geometry, IsFiniteVec2)
{
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_TRUE(is_finite(Vec2{1.0f, -2.0f}));
    EXPECT_FALSE(is_finite(Vec2{nan, 1.0f}));
    EXPECT_FALSE(is_finite(Vec2{1.0f, inf}));
    EXPECT_FALSE(is_finite(Vec2{nan, nan}));
}

// S1.1: exact collinearity and on-segment classification.
TEST(Geometry, PointOnSegmentExact)
{
    // Collinear: interior, endpoint, and past the endpoint.
    EXPECT_TRUE(point_on_segment_exact(Vec2{2.0f, 0.0f}, Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}));
    EXPECT_TRUE(point_on_segment_exact(Vec2{4.0f, 0.0f}, Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}));
    EXPECT_FALSE(point_on_segment_exact(Vec2{5.0f, 0.0f}, Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}));
    // Not collinear.
    EXPECT_FALSE(point_on_segment_exact(Vec2{2.0f, 1.0f}, Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}));
    // Diagonal segment.
    EXPECT_TRUE(point_on_segment_exact(Vec2{1.0f, 1.0f}, Vec2{0.0f, 0.0f}, Vec2{2.0f, 2.0f}));
    // Degenerate segment (a == b): true only at the point itself.
    EXPECT_TRUE(point_on_segment_exact(Vec2{1.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{1.0f, 0.0f}));
    EXPECT_FALSE(point_on_segment_exact(Vec2{2.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{1.0f, 0.0f}));
}

// S1.1: distance to a closed segment (interior foot, endpoints, degenerate).
TEST(Geometry, PointSegmentDistance)
{
    // Perpendicular foot inside the segment.
    EXPECT_FLOAT_EQ(
        point_segment_distance(Vec2{2.0f, 3.0f}, Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}), 3.0f);
    // Projection outside: distance to the nearest endpoint.
    EXPECT_FLOAT_EQ(
        point_segment_distance(Vec2{5.0f, 0.0f}, Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}), 1.0f);
    EXPECT_FLOAT_EQ(
        point_segment_distance(Vec2{-1.0f, 0.0f}, Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}), 1.0f);
    // Degenerate segment (a == b): plain point distance.
    EXPECT_FLOAT_EQ(
        point_segment_distance(Vec2{5.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{1.0f, 0.0f}), 4.0f);
}
