#pragma once

// Internal geometric predicates shared by triangulation, mesh construction, and
// pathfinding. Header-only, allocation-free, deterministic. All tests compute in
// double precision for robustness; the public contract measures distances in float
// metres with `kEpsilon` as the geometric tolerance (see docs/requirements/mesh.md).

#include <vwmini/geometry.hpp>

#include <algorithm>
#include <cmath>

namespace vwmini::internal {

/** Geometric tolerance in metres (MSH numerical policy). */
inline constexpr float kEpsilon = 1e-4f;
/** Double form of `kEpsilon` for internal computation. */
inline constexpr double kEpsD = 1e-4;
/** A triangle/outline corner is non-degenerate when |signed double area| exceeds this. */
inline constexpr double kMinArea = double(kEpsilon) * double(kEpsilon);

/** True when `value` is a finite float (not NaN, not infinite). */
[[nodiscard]] inline bool is_finite(float value) noexcept
{
    return std::isfinite(value);
}

/** True when both components of `point` are finite. */
[[nodiscard]] inline bool is_finite(Vec2 point) noexcept
{
    return std::isfinite(point.x) && std::isfinite(point.y);
}

/**
 * Signed doubled area / orientation of the ordered triple (a, b, c).
 * Positive when c lies to the left of the directed line a->b (CCW turn),
 * negative to the right (CW), zero when collinear. Computed in double.
 */
[[nodiscard]] inline double orient(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    // Widen to double BEFORE subtracting: float-rounded differences would widen
    // the ambiguous band near zero and make near-degenerate classification
    // input-order-sensitive (the architecture specifies a double cross product).
    return (double(b.x) - double(a.x)) * (double(c.y) - double(a.y)) -
           (double(b.y) - double(a.y)) * (double(c.x) - double(a.x));
}

/** True when the triangle (a, b, c) has non-degenerate area per the mesh policy. */
[[nodiscard]] inline bool non_degenerate(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return std::abs(orient(a, b, c)) > kMinArea;
}

/** Squared Euclidean distance between two points, in double. */
[[nodiscard]] inline double distance_sq(Vec2 a, Vec2 b) noexcept
{
    const double dx = double(a.x) - double(b.x);
    const double dy = double(a.y) - double(b.y);
    return dx * dx + dy * dy;
}

/** Euclidean distance between two points, in double. */
[[nodiscard]] inline double distance(Vec2 a, Vec2 b) noexcept
{
    return std::sqrt(distance_sq(a, b));
}

/** True when `a` and `b` are within `kEpsilon` of each other. */
[[nodiscard]] inline bool near(Vec2 a, Vec2 b) noexcept
{
    return distance(a, b) <= kEpsD;
}

/** Euclidean distance from point `p` to the closed segment [a, b], in double. */
[[nodiscard]] inline double distance_point_segment(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    const double abx = double(b.x) - double(a.x);
    const double aby = double(b.y) - double(a.y);
    const double len2 = abx * abx + aby * aby;
    if (len2 <= 0.0) {
        return distance(p, a);
    }
    const double apx = double(p.x) - double(a.x);
    const double apy = double(p.y) - double(a.y);
    const double t = std::clamp((apx * abx + apy * aby) / len2, 0.0, 1.0);
    // Offset from the closest point a + t*ab to p; stays in double throughout.
    const double dx = apx - t * abx;
    const double dy = apy - t * aby;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * True when point `p` lies within the closed triangle (a, b, c) including its
 * boundary, using exact orientation signs. Assumes a CCW triangle.
 */
[[nodiscard]] inline bool point_in_triangle_closed(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    const double d0 = orient(a, b, p);
    const double d1 = orient(b, c, p);
    const double d2 = orient(c, a, p);
    return d0 >= 0.0 && d1 >= 0.0 && d2 >= 0.0;
}

/**
 * True when point `p` lies strictly inside the CCW triangle (a, b, c),
 * excluding all edges and vertices, using exact orientation signs.
 */
[[nodiscard]] inline bool point_in_triangle_strict(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return orient(a, b, p) > 0.0 && orient(b, c, p) > 0.0 && orient(c, a, p) > 0.0;
}

/**
 * True when the closed segments [a, b] and [c, d] cross transversally at a point
 * strictly interior to both (a proper crossing). Excludes collinear overlap and
 * endpoint touching, so it flags genuine interior overlap between two triangles
 * and never a shared edge or shared vertex.
 */
[[nodiscard]] inline bool segments_properly_cross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    const double d1 = orient(c, d, a);
    const double d2 = orient(c, d, b);
    const double d3 = orient(a, b, c);
    const double d4 = orient(a, b, d);
    const bool straddle_cd = (d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0);
    const bool straddle_ab = (d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0);
    return straddle_cd && straddle_ab;
}

/**
 * True when the closed segments [a, b] and [c, d] share at least one point
 * (proper crossing, or touching/overlap in the collinear case). Uses exact
 * double orientation signs; inputs within `kEpsilon` are out of the
 * interoperability guarantee, so exact tests remain safe and deterministic.
 */
[[nodiscard]] inline bool segments_intersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    if (segments_properly_cross(a, b, c, d)) {
        return true;
    }
    // Collinear / endpoint-touching cases: an endpoint on the other segment counts.
    const auto on = [](Vec2 p, Vec2 q, Vec2 r) {
        // r collinear with pq (orient == 0) and within the bounding box of pq.
        return orient(p, q, r) == 0.0 && std::min(p.x, q.x) <= r.x && r.x <= std::max(p.x, q.x) &&
               std::min(p.y, q.y) <= r.y && r.y <= std::max(p.y, q.y);
    };
    return on(c, d, a) || on(c, d, b) || on(a, b, c) || on(a, b, d);
}

/**
 * True when point `p` is contained by the closed CCW triangle (a, b, c) under the
 * MSH-006 boundary policy: strictly inside, on the boundary, or within Euclidean
 * distance `kEpsilon` of one of the three closed edge segments.
 */
[[nodiscard]] inline bool triangle_contains_eps(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return point_in_triangle_closed(p, a, b, c) || distance_point_segment(p, a, b) <= kEpsD ||
           distance_point_segment(p, b, c) <= kEpsD || distance_point_segment(p, c, a) <= kEpsD;
}

} // namespace vwmini::internal
