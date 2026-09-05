#pragma once

#include <vwmini/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>

namespace vwmini::detail {

// Geometric tolerance (metres).
inline constexpr float epsilon = 1e-4f;
inline constexpr float epsilon_sq = epsilon * epsilon; // 1e-8

// True when both coordinates are finite (not NaN / inf).
inline bool finite(Vec2 v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y);
}

// True when the scalar is finite (not NaN / inf).
inline bool finite(float x) noexcept
{
    return std::isfinite(x);
}

// Twice the signed area of the triangle (a, b, c) (positive iff CCW).
inline float signed_area(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return cross(b - a, c - a);
}

// True when the triangle has strictly positive (CCW) signed area
// greater than epsilon^2.
inline bool valid_triangle(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return signed_area(a, b, c) > epsilon_sq;
}

// Signed area of a polygon ring (shoelace, 0.5 * sum cross). Positive iff CCW.
inline float signed_polygon_area(std::span<const Vec2> v) noexcept
{
    const int n = static_cast<int>(v.size());
    float area = 0.0f;
    for (int i = 0; i < n; ++i) {
        const Vec2 a = v[i];
        const Vec2 b = v[(i + 1) % n];
        area += cross(a, b);
    }
    return 0.5f * area;
}

// True when the ring is CCW (signed area beyond eps) and strictly convex: no
// reflex vertex and at least one strictly convex corner.
inline bool is_convex_ccw(std::span<const Vec2> v) noexcept
{
    if (signed_polygon_area(v) <= epsilon_sq) {
        return false;
    }
    const int n = static_cast<int>(v.size());
    if (n < 3) {
        return false;
    }
    bool saw_convex = false;
    for (int i = 0; i < n; ++i) {
        const Vec2 a = v[i];
        const Vec2 b = v[(i + 1) % n];
        const Vec2 c = v[(i + 2) % n];
        const float cr = cross(b - a, c - b);
        if (cr < -epsilon) {
            return false; // reflex vertex
        }
        if (cr > epsilon) {
            saw_convex = true;
        }
    }
    return saw_convex;
}

// Strict point-in-triangle test (boundary excluded). p strictly inside iff all
// edge cross products are strictly positive.
inline bool point_in_triangle_strict(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    const float c0 = cross(b - a, p - a);
    const float c1 = cross(c - b, p - b);
    const float c2 = cross(a - c, p - c);
    return c0 > 0.0f && c1 > 0.0f && c2 > 0.0f;
}

// True when p is inside the closed triangle (edges included), using the small
// negative tolerance only to absorb floating noise on shared edges.
inline bool point_in_triangle_closed(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    const float c0 = cross(b - a, p - a);
    const float c1 = cross(c - b, p - b);
    const float c2 = cross(a - c, p - c);
    return c0 >= -epsilon && c1 >= -epsilon && c2 >= -epsilon;
}

// Squared distance from p to the closed segment [a, b].
inline float dist_sq_point_segment(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    const Vec2 ab = b - a;
    const float ab_len_sq = dot(ab, ab);
    if (ab_len_sq <= 0.0f) {
        return dot(p - a, p - a);
    }
    float t = dot(p - a, ab) / ab_len_sq;
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    const Vec2 proj = a + ab * t;
    return dot(p - proj, p - proj);
}

// Squared Euclidean distance from p to the closed segment [a, b].
inline float dist_point_segment(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    return std::sqrt(dist_sq_point_segment(p, a, b));
}

// True when vertex p lies in the interior of segment (a,b) within epsilon
// (a T-junction: not within epsilon of either endpoint).
inline bool vertex_in_segment_interior(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    if (dist_point_segment(p, a, b) > epsilon) {
        return false;
    }
    if (dot(p - a, p - a) < epsilon_sq) {
        return false; // near endpoint a
    }
    if (dot(p - b, p - b) < epsilon_sq) {
        return false; // near endpoint b
    }
    return true;
}

// True when point p is in triangle `a,b,c` under the boundary policy (MSH-006):
// strictly inside, or within epsilon of any closed edge.
inline bool contains_point_triangle(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    if (point_in_triangle_strict(p, a, b, c)) {
        return true;
    }
    if (dist_sq_point_segment(p, a, b) <= epsilon_sq) {
        return true;
    }
    if (dist_sq_point_segment(p, b, c) <= epsilon_sq) {
        return true;
    }
    if (dist_sq_point_segment(p, c, a) <= epsilon_sq) {
        return true;
    }
    return false;
}

// Proper segment intersection of (a,b) and (c,d): they cross strictly, not just
// touch at shared endpoints. Returns the intersection parameter on (c,d) via out.
inline bool proper_segment_intersection(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    const Vec2 r = b - a;
    const Vec2 s = d - c;
    const float rs = cross(r, s);
    if (std::fabs(rs) <= epsilon_sq) {
        return false; // parallel or degenerate
    }
    const Vec2 qp = c - a;
    const float t = cross(qp, s) / rs;
    const float u = cross(qp, r) / rs;
    if (t <= 0.0f || t >= 1.0f || u <= 0.0f || u >= 1.0f) {
        return false;
    }
    return true;
}

} // namespace vwmini::detail
