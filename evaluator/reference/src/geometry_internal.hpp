#pragma once

// Private reference-only geometry predicates. Never installed; only the canonical
// headers in candidate/include/vwmini/ are public. These helpers are shared by the
// geometry module now and the mesh/path modules later.

#include <vwmini/geometry.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace vwmini::geom {

/// Geometric tolerance in metres (double form of the public 1e-4f epsilon).
constexpr double kEpsilon = 1e-4;
/// Squared tolerance for area degeneracy: a triangle is non-degenerate when its
/// absolute signed double area is greater than kEpsilon * kEpsilon.
constexpr double kEpsilonSquared = kEpsilon * kEpsilon;

/// True when both coordinates are finite.
[[nodiscard]] inline bool is_finite(Vec2 v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y);
}

/// Twice the signed area of the triangle (a, b, c); positive for a CCW turn.
/// Uses double intermediates so float-coordinate products are effectively exact.
[[nodiscard]] inline double orient(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    const double ax = a.x, ay = a.y;
    const double bx = b.x, by = b.y;
    const double cx = c.x, cy = c.y;
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

/// Twice the signed area of an ordered ring; positive when counter-clockwise.
[[nodiscard]] double signed_area2(const std::vector<Vec2>& pts) noexcept;

/// True when p lies inside or on the counter-clockwise triangle (a, b, c).
/// Inclusive boundary is the conservative choice for ear tests and containment.
[[nodiscard]] inline bool point_in_triangle_ccw(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return orient(a, b, p) >= 0.0 && orient(b, c, p) >= 0.0 && orient(c, a, p) >= 0.0;
}

} // namespace vwmini::geom
