// Private geometry helpers shared by triangulation, mesh, path and avoidance.
// No state, no allocation; all predicates are deterministic in float.
#pragma once

#include <vwmini/geometry.hpp>

#include <cmath>
#include <cstddef>
#include <expected>
#include <string_view>

namespace vwmini::detail {

// Builds the diagnosed error for `return fail(...)` statements.
[[nodiscard]] inline std::unexpected<Error> fail(ErrorCode code, std::string_view message)
{
    return std::unexpected(Error{code, std::string{message}});
}

inline constexpr float epsilon = 1e-4f;
inline constexpr float epsilon2 = epsilon * epsilon;

[[nodiscard]] constexpr bool is_finite(Vec2 v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y);
}

// Strict interior test against a CCW triangle (exact float signs).
[[nodiscard]] constexpr bool strictly_inside(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return cross(b - a, p - a) > 0.0f && cross(c - b, p - b) > 0.0f &&
           cross(a - c, p - c) > 0.0f;
}

// Signed double area of a triangle: cross(b - a, c - a).
// Positive for counter-clockwise (CCW) ordering.
[[nodiscard]] constexpr float triangle_area2(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return cross(b - a, c - a);
}

// Signed area of a simple polygon (shoelace), CCW positive.
[[nodiscard]] float polygon_area(const Polygon& p) noexcept;

// Distance from point p to the closed segment [a, b]; finite inputs,
// possibly degenerate segment (a == b).
[[nodiscard]] float point_segment_distance(Vec2 p, Vec2 a, Vec2 b) noexcept;

// True when p lies on the closed segment [a, b] (exact float arithmetic;
// used only where the spec prescribes tolerance, with an explicit bound).
[[nodiscard]] bool point_on_segment_exact(Vec2 p, Vec2 a, Vec2 b) noexcept;

// True when segment [a, b] and segment [c, d] cross in their interiors.
// Endpoint touches and collinear overlaps return false (see
// segments_collinear_overlap for the collinear case).
[[nodiscard]] bool segments_properly_intersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept;

// True when segment [a, b] and segment [c, d] are collinear and their
// projections overlap with non-zero length (excludes mere endpoint touch).
[[nodiscard]] bool segments_collinear_overlap(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept;

// Exact squared distance, avoiding sqrt on hot paths.
[[nodiscard]] constexpr float squared_distance(Vec2 a, Vec2 b) noexcept
{
    return dot(b - a, b - a);
}

} // namespace vwmini::detail
