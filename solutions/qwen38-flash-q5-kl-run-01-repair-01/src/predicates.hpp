#pragma once

// Internal geometric primitives, and the single place that encodes the contract's
// numerical policy.
//
// Numeric policy (see architecture.md "Numeric policy"):
//   * Every decision about validity, incidence, containment, segment coverage, topology or a
//     route is computed in double intermediates through the helpers below. A float product of
//     two floats is exact in double, and double covers a far wider exponent range, so signs,
//     comparisons and accumulations stay meaningful for every accepted finite float input —
//     where float multiplication would overflow to infinity or cancel to the wrong sign.
//   A decision is never allowed to come from an overflowed, infinite or NaN intermediate.
//   Consequence: internal code must not subtract, multiply or accumulate two public `Vec2`
//   values before handing them to a helper, because that arithmetic is float. Every operand is
//   widened on its own (implicitly to `Vec2d`) and the arithmetic happens in double.
//   Epsilon comparisons keep the resolution of the stored float coordinates: above roughly
//   1e3 m from the origin a float ULP exceeds epsilon, so an epsilon test there degenerates to
//   exact equality. Containment, incidence and routing stay exact at any accepted coordinate.
//   `epsilon` (1e-4 m) is used where the contract names it: point/edge distance, complete-edge
//   endpoint matching and waypoint omission (SIM-002). It is reused, deliberately, for the two
//   route-state rules that measure that same metre scale: dropping a waypoint the agent already
//   occupies (`agent.hpp`) and dropping one it has reached (`simulation.cpp`). There is no
//   orientation or area tolerance: a triangle is non-degenerate exactly when
//   |signed double area| > epsilon * epsilon (mesh.md).

#include "vec2d.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <vector>

namespace vwmini::detail
{

/// Geometric tolerance in metres.
inline constexpr float kEpsilon = 1e-4f;

/// Minimum absolute signed double area for a triangle to be non-degenerate.
inline constexpr double kMinNonDegenerateDoubleArea =
    static_cast<double>(kEpsilon) * static_cast<double>(kEpsilon);

/// `epsilon` as a double length, for comparisons against double distances.
inline constexpr double kEpsilonLength = static_cast<double>(kEpsilon);

/// Squared `epsilon`, for squared-distance comparisons in the policy's double domain.
/// Deliberately a separate constant from the degenerate-area threshold above: a change to the
/// area rule must not silently change the distance rule, even though both derive from epsilon.
inline constexpr double kEpsilonSquared =
    static_cast<double>(kEpsilon) * static_cast<double>(kEpsilon);

[[nodiscard]] inline bool is_finite(float value) noexcept
{
    return std::isfinite(value);
}

[[nodiscard]] inline bool is_finite(Vec2d value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y);
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

[[nodiscard]] constexpr double dot_d(Vec2d a, Vec2d b) noexcept
{
    return a.x * b.x + a.y * b.y;
}

[[nodiscard]] constexpr double cross_d(Vec2d a, Vec2d b) noexcept
{
    return a.x * b.y - a.y * b.x;
}

[[nodiscard]] constexpr double length_squared(Vec2d value) noexcept
{
    return dot_d(value, value);
}

[[nodiscard]] inline double magnitude(Vec2d value) noexcept
{
    return std::hypot(value.x, value.y);
}

/// Euclidean distance from `a` to `b` in the policy's double domain. `hypot` scales internally,
/// so a finite input never produces an infinite result.
[[nodiscard]] inline double distance(Vec2d a, Vec2d b) noexcept
{
    return magnitude(b - a);
}

/// Signed double area of triangle `a,b,c`; positive for counter-clockwise winding.
[[nodiscard]] constexpr double signed_double_area(Vec2d a, Vec2d b, Vec2d c) noexcept
{
    return cross_d(b - a, c - a);
}

/// True when `p` is strictly inside the counter-clockwise triangle `a,b,c`.
[[nodiscard]] constexpr bool strictly_inside_triangle(Vec2d p, Vec2d a, Vec2d b, Vec2d c) noexcept
{
    return signed_double_area(a, b, p) > 0.0 && signed_double_area(b, c, p) > 0.0 &&
           signed_double_area(c, a, p) > 0.0;
}

/// Squared distance from `p` to the closed segment `a`..`b`.
[[nodiscard]] inline double distance_squared_to_segment(Vec2d p, Vec2d a, Vec2d b) noexcept
{
    const Vec2d edge = b - a;
    const double edge_length2 = dot_d(edge, edge);
    if (edge_length2 == 0.0)
    {
        return length_squared(p - a);
    }
    const double along = std::clamp(dot_d(p - a, edge) / edge_length2, 0.0, 1.0);
    return length_squared(p - (a + edge * along));
}

/// True when `p` lies on the closed segment `a`..`b` within `tolerance`.
[[nodiscard]] inline bool point_on_segment(Vec2d p, Vec2d a, Vec2d b, double tolerance) noexcept
{
    return distance_squared_to_segment(p, a, b) <= tolerance * tolerance;
}

/**
 * Unit vector in the direction of `value`, computed entirely in the policy's double domain.
 *
 * The scale is divided out through the largest component first, so a vector joining two
 * extreme coordinates normalizes correctly instead of overflowing. Only a zero or non-finite
 * vector yields a zero vector.
 */
[[nodiscard]] inline Vec2d unit(Vec2d value) noexcept
{
    const double largest = std::max(std::abs(value.x), std::abs(value.y));
    if (largest == 0.0 || !is_finite(value))
    {
        return Vec2d{};
    }
    const Vec2d scaled{value.x / largest, value.y / largest};
    return scaled / magnitude(scaled);
}

/**
 * Append every parameter in `[0,1]` at which segment `p`->`q` meets segment `a`->`b`.
 *
 * A transversal crossing appends one value. Collinear overlap appends both ends of the shared
 * span: a caller decomposing its own segment into covered intervals needs every boundary, and
 * missing one can straddle covered and uncovered ground in a single sample. Misses and
 * parameters outside `[0,1]` append nothing.
 */
inline void append_crossing_parameters(Vec2d p, Vec2d q, Vec2d a, Vec2d b,
                                       std::vector<double> &cuts) noexcept
{
    const Vec2d d1 = q - p;
    const Vec2d d2 = b - a;
    const Vec2d offset = a - p;
    const double denominator = cross_d(d1, d2);

    if (denominator != 0.0)
    {
        const double t = cross_d(offset, d2) / denominator;
        const double s = cross_d(offset, d1) / denominator;
        if (t >= 0.0 && t <= 1.0 && s >= 0.0 && s <= 1.0)
        {
            cuts.push_back(t);
        }
        return;
    }

    // Parallel: only collinear segments overlap.
    if (cross_d(offset, d1) != 0.0)
    {
        return;
    }
    const double length2 = dot_d(d1, d1);
    if (length2 == 0.0)
    {
        return;
    }
    const double ta = dot_d(a - p, d1) / length2;
    const double tb = dot_d(b - p, d1) / length2;
    const double overlap_start = std::max(0.0, std::min(ta, tb));
    const double overlap_end = std::min(1.0, std::max(ta, tb));
    if (overlap_start > overlap_end)
    {
        return;
    }
    cuts.push_back(overlap_start);
    if (overlap_end != overlap_start)
    {
        cuts.push_back(overlap_end);
    }
}

/// True when the two segments cross at a point strictly inside both of them.
[[nodiscard]] constexpr bool segments_cross_internally(Vec2d a1, Vec2d a2, Vec2d b1,
                                                       Vec2d b2) noexcept
{
    const double first = signed_double_area(a1, a2, b1);
    const double second = signed_double_area(a1, a2, b2);
    const double third = signed_double_area(b1, b2, a1);
    const double fourth = signed_double_area(b1, b2, a2);
    const bool ab_splits_cd = (third > 0.0 && fourth < 0.0) || (third < 0.0 && fourth > 0.0);
    const bool cd_splits_ab = (first > 0.0 && second < 0.0) || (first < 0.0 && second > 0.0);
    return ab_splits_cd && cd_splits_ab;
}

} // namespace vwmini::detail
