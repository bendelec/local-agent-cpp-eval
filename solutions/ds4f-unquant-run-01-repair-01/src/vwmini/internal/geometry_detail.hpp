#pragma once

// Private geometric predicates shared by the geometry, nav_mesh, and path
// modules. Pure, allocation-free, stateless. Geometric arithmetic is done in
// double for stability; the public API stays float.
//
// Tolerance policy (per requirements): `epsilon` applies only to point/edge
// distance (MSH-006) and shared-edge endpoint matching (MSH-005). Orientation
// and incidence predicates are exact in double. `epsilon * epsilon` is the
// triangle degeneracy threshold (MSH-002/MSH-004).

#include <vwmini/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace vwmini::detail {

inline constexpr float kEpsilon = 1e-4f;
inline constexpr double kEpsilonD = 1e-4;

[[nodiscard]] inline bool is_finite(Vec2 v) noexcept {
  return std::isfinite(v.x) && std::isfinite(v.y);
}

[[nodiscard]] inline bool is_finite(float value) noexcept {
  return std::isfinite(value);
}

[[nodiscard]] inline double cross2(Vec2 a, Vec2 b) noexcept {
  return static_cast<double>(a.x) * static_cast<double>(b.y) -
         static_cast<double>(a.y) * static_cast<double>(b.x);
}

[[nodiscard]] inline double dot2(Vec2 a, Vec2 b) noexcept {
  return static_cast<double>(a.x) * static_cast<double>(b.x) +
         static_cast<double>(a.y) * static_cast<double>(b.y);
}

/// Signed double area of triangle (a, b, c); positive when CCW.
[[nodiscard]] inline double signed_area2(Vec2 a, Vec2 b, Vec2 c) noexcept {
  return cross2(b - a, c - a);
}

/// Orientation sign: +1 CCW, -1 CW, 0 collinear (exact double comparison).
[[nodiscard]] inline int orient(Vec2 a, Vec2 b, Vec2 c) noexcept {
  const double s = signed_area2(a, b, c);
  return (s > 0.0) ? 1 : (s < 0.0 ? -1 : 0);
}

/// Squared Euclidean distance from p to the closed segment [a, b].
[[nodiscard]] inline double dist2_point_segment(Vec2 p, Vec2 a,
                                                Vec2 b) noexcept {
  const double abx = static_cast<double>(b.x) - a.x;
  const double aby = static_cast<double>(b.y) - a.y;
  const double apx = static_cast<double>(p.x) - a.x;
  const double apy = static_cast<double>(p.y) - a.y;
  const double len2 = abx * abx + aby * aby;
  if (len2 == 0.0) {
    return apx * apx + apy * apy; // degenerate segment; safe fallback
  }
  const double t = std::clamp((apx * abx + apy * aby) / len2, 0.0, 1.0);
  const double cx = apx - t * abx;
  const double cy = apy - t * aby;
  return cx * cx + cy * cy;
}

/// Euclidean distance from p to the closed segment [a, b].
[[nodiscard]] inline double dist_point_segment(Vec2 p, Vec2 a,
                                               Vec2 b) noexcept {
  return std::sqrt(dist2_point_segment(p, a, b));
}

/// True when p lies exactly on the closed segment [a, b].
[[nodiscard]] inline bool point_on_segment(Vec2 p, Vec2 a, Vec2 b) noexcept {
  return orient(a, b, p) == 0 && dist2_point_segment(p, a, b) == 0.0;
}

/// True when the closed segments [a,b] and [c,d] intersect (including touches).
[[nodiscard]] inline bool segments_intersect(Vec2 a, Vec2 b, Vec2 c,
                                             Vec2 d) noexcept {
  const int o1 = orient(a, b, c);
  const int o2 = orient(a, b, d);
  const int o3 = orient(c, d, a);
  const int o4 = orient(c, d, b);
  if (o1 == 0 && point_on_segment(c, a, b))
    return true;
  if (o2 == 0 && point_on_segment(d, a, b))
    return true;
  if (o3 == 0 && point_on_segment(a, c, d))
    return true;
  if (o4 == 0 && point_on_segment(b, c, d))
    return true;
  return (o1 * o2 < 0) && (o3 * o4 < 0);
}

/// True when segments [a,b] and [c,d] cross at an interior point (touching
/// endpoints or collinear overlap does not count).
[[nodiscard]] inline bool proper_cross(Vec2 a, Vec2 b, Vec2 c,
                                       Vec2 d) noexcept {
  const int o1 = orient(a, b, c);
  const int o2 = orient(a, b, d);
  const int o3 = orient(c, d, a);
  const int o4 = orient(c, d, b);
  return (o1 * o2 < 0) && (o3 * o4 < 0);
}

/// Strict point-in-triangle test; assumes (a, b, c) is CCW.
[[nodiscard]] inline bool point_in_triangle_strict(Vec2 p, Vec2 a, Vec2 b,
                                                   Vec2 c) noexcept {
  return signed_area2(a, b, p) > 0.0 && signed_area2(b, c, p) > 0.0 &&
         signed_area2(c, a, p) > 0.0;
}

/// Closed point-in-triangle test (boundary included); assumes (a, b, c) is CCW.
[[nodiscard]] inline bool point_in_triangle_closed(Vec2 p, Vec2 a, Vec2 b,
                                                   Vec2 c) noexcept {
  return signed_area2(a, b, p) >= 0.0 && signed_area2(b, c, p) >= 0.0 &&
         signed_area2(c, a, p) >= 0.0;
}

/// MSH-006 containment: strictly inside, or within `epsilon` of a closed edge.
[[nodiscard]] inline bool point_contained_by_triangle(Vec2 p, Vec2 a, Vec2 b,
                                                      Vec2 c) noexcept {
  if (point_in_triangle_strict(p, a, b, c)) {
    return true;
  }
  return dist_point_segment(p, a, b) <= kEpsilonD ||
         dist_point_segment(p, b, c) <= kEpsilonD ||
         dist_point_segment(p, c, a) <= kEpsilonD;
}

/// True when (a, b, c) is CCW and its double area exceeds `epsilon * epsilon`.
[[nodiscard]] inline bool is_valid_ccw_triangle(Vec2 a, Vec2 b,
                                                Vec2 c) noexcept {
  return signed_area2(a, b, c) > kEpsilonD * kEpsilonD;
}

/// Closest point of the closed segment [a, b] to p.
[[nodiscard]] inline Vec2 closest_point_on_segment(Vec2 p, Vec2 a,
                                                   Vec2 b) noexcept {
  const double abx = static_cast<double>(b.x) - a.x;
  const double aby = static_cast<double>(b.y) - a.y;
  const double apx = static_cast<double>(p.x) - a.x;
  const double apy = static_cast<double>(p.y) - a.y;
  const double len2 = abx * abx + aby * aby;
  const double t = (len2 == 0.0)
                       ? 0.0
                       : std::clamp((apx * abx + apy * aby) / len2, 0.0, 1.0);
  return {static_cast<float>(a.x + t * abx), static_cast<float>(a.y + t * aby)};
}

/// Shoelace signed area of a polygon ring; positive when CCW.
[[nodiscard]] inline double polygon_signed_area(const Polygon &poly) noexcept {
  double sum = 0.0;
  const std::size_t n = poly.vertices.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 a = poly.vertices[i];
    const Vec2 b = poly.vertices[(i + 1) % n];
    sum += cross2(a, b);
  }
  return 0.5 * sum;
}

} // namespace vwmini::detail
