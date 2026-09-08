#pragma once

// Internal geometric primitives, and the single place that encodes the contract's
// numerical policy: `epsilon` (1e-4 m) applies to point/edge distance and shared-edge
// endpoint matching; a triangle is non-degenerate when the absolute value of its signed
// double area exceeds epsilon * epsilon (docs/requirements/mesh.md).

#include <vwmini/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <optional>

namespace vwmini::detail
{

/// Geometric tolerance in metres.
inline constexpr float kEpsilon = 1e-4f;

/// Minimum absolute signed double area for a triangle to be non-degenerate.
inline constexpr float kMinNonDegenerateDoubleArea = kEpsilon * kEpsilon;

[[nodiscard]] inline bool is_finite(float value) noexcept
{
    return std::isfinite(value);
}

[[nodiscard]] inline bool is_finite(Vec2 value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y);
}

[[nodiscard]] inline bool all_finite(std::initializer_list<float> values) noexcept
{
    for (const float value : values)
    {
        if (!is_finite(value))
        {
            return false;
        }
    }
    return true;
}

[[nodiscard]] inline float distance(Vec2 a, Vec2 b) noexcept
{
    return length(a - b);
}

[[nodiscard]] constexpr float length_squared(Vec2 value) noexcept
{
    return dot(value, value);
}

/// Signed double area of triangle `a,b,c`; positive for counter-clockwise winding.
[[nodiscard]] constexpr float signed_double_area(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return cross(b - a, c - a);
}

/// True when `p` is strictly inside the counter-clockwise triangle `a,b,c`.
[[nodiscard]] constexpr bool strictly_inside_triangle(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return signed_double_area(a, b, p) > 0.0f && signed_double_area(b, c, p) > 0.0f &&
           signed_double_area(c, a, p) > 0.0f;
}

/// Squared distance from `p` to the closed segment `a`..`b`.
[[nodiscard]] inline float distance_squared_to_segment(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    const Vec2 edge = b - a;
    const float edge_length2 = dot(edge, edge);
    if (edge_length2 == 0.0f)
    {
        return length_squared(p - a);
    }
    const float along = std::clamp(dot(p - a, edge) / edge_length2, 0.0f, 1.0f);
    return length_squared(p - (a + edge * along));
}

/// True when `p` lies on the closed segment `a`..`b` within `tolerance`.
[[nodiscard]] inline bool point_on_segment(Vec2 p, Vec2 a, Vec2 b, float tolerance) noexcept
{
    return distance_squared_to_segment(p, a, b) <= tolerance * tolerance;
}

/**
 * Parameter `t` in `[0,1]` where segment `p`->`q` meets segment `a`->`b`, if they meet.
 *
 * Collinear overlap reports the first common parameter. Used to collect the critical
 * parameters of a segment, where reporting an extra value costs nothing.
 */
[[nodiscard]] inline std::optional<float> segment_intersection_parameter(Vec2 p, Vec2 q, Vec2 a,
                                                                         Vec2 b) noexcept
{
    const Vec2 d1 = q - p;
    const Vec2 d2 = b - a;
    const Vec2 offset = a - p;
    const float denominator = cross(d1, d2);

    if (denominator != 0.0f)
    {
        const float t = cross(offset, d2) / denominator;
        const float s = cross(offset, d1) / denominator;
        if (t >= 0.0f && t <= 1.0f && s >= 0.0f && s <= 1.0f)
        {
            return t;
        }
        return std::nullopt;
    }

    // Parallel: only collinear segments overlap.
    if (cross(offset, d1) != 0.0f)
    {
        return std::nullopt;
    }
    const float length2 = dot(d1, d1);
    if (length2 == 0.0f)
    {
        return std::nullopt;
    }
    const float ta = dot(a - p, d1) / length2;
    const float tb = dot(b - p, d1) / length2;
    const float overlap_start = std::max(0.0f, std::min(ta, tb));
    const float overlap_end = std::min(1.0f, std::max(ta, tb));
    if (overlap_start <= overlap_end)
    {
        return overlap_start;
    }
    return std::nullopt;
}

/// True when the two segments cross at a point strictly inside both of them.
[[nodiscard]] inline bool segments_cross_internally(Vec2 a1, Vec2 a2, Vec2 b1, Vec2 b2) noexcept
{
    const float first = signed_double_area(a1, a2, b1);
    const float second = signed_double_area(a1, a2, b2);
    const float third = signed_double_area(b1, b2, a1);
    const float fourth = signed_double_area(b1, b2, a2);
    const bool ab_splits_cd = (third > 0.0f && fourth < 0.0f) || (third < 0.0f && fourth > 0.0f);
    const bool cd_splits_ab = (first > 0.0f && second < 0.0f) || (first < 0.0f && second > 0.0f);
    return ab_splits_cd && cd_splits_ab;
}

} // namespace vwmini::detail
