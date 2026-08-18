// Geometry module: Vec2 vector operations and deterministic simple-polygon
// triangulation (MSH-001..MSH-003).
//
// Plan:
//   - `length` / `normalized`: plain vector math; normalize of zero (or any non-
//     positive length, which also covers NaN) returns {0,0}.
//   - `triangulate_simple_polygon`: validate finiteness, vertex count, cyclic
//     consecutive duplicates, CCW non-degenerate winding, and self-intersections;
//     then remove exact-collinear vertices and run deterministic ear clipping that
//     always clips the first valid ear in original cyclic order.
//   - Double intermediates for all predicates; float only at the public boundary.
//   - No global mutable state; helpers are free functions in an anonymous namespace.

#include "geometry_internal.hpp"

#include <vwmini/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace vwmini {

namespace geom {

[[nodiscard]] double signed_area2(const std::vector<Vec2>& pts) noexcept
{
    double sum = 0.0;
    const std::size_t n = pts.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 p = pts[i];
        const Vec2 q = pts[(i + 1u) % n];
        sum += static_cast<double>(p.x) * static_cast<double>(q.y) -
               static_cast<double>(p.y) * static_cast<double>(q.x);
    }
    return sum;
}

} // namespace geom

[[nodiscard]] float length(Vec2 value) noexcept
{
    return std::sqrt(dot(value, value));
}

[[nodiscard]] Vec2 normalized(Vec2 value) noexcept
{
    const float len = length(value);
    // Zero input (and, defensively, NaN) yields the zero vector.
    if (!(len > 0.0f)) {
        return {};
    }
    return value * (1.0f / len);
}

namespace {

using geom::orient;

Error fail(ErrorCode code, std::string message)
{
    return Error{code, std::move(message)};
}

/// True when p lies on the closed segment [a, b] using exact double comparisons.
/// Callers establish collinearity first, so a bounding-box test suffices here.
bool on_segment_exact(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    const double px = p.x, py = p.y;
    const double ax = a.x, ay = a.y;
    const double bx = b.x, by = b.y;
    return px >= std::min(ax, bx) && px <= std::max(ax, bx) &&
           py >= std::min(ay, by) && py <= std::max(ay, by);
}

/// True when the closed segments [a,b] and [c,d] intersect: a proper crossing, an
/// endpoint of one lying on the other, or a collinear overlap. Used only for the
/// outline self-intersection test, so exact (epsilon-free) comparisons are correct.
bool segments_intersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    const double o1 = orient(a, b, c);
    const double o2 = orient(a, b, d);
    const double o3 = orient(c, d, a);
    const double o4 = orient(c, d, b);

    if (((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) &&
        ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0))) {
        return true;
    }
    if (o1 == 0.0 && on_segment_exact(c, a, b)) return true;
    if (o2 == 0.0 && on_segment_exact(d, a, b)) return true;
    if (o3 == 0.0 && on_segment_exact(a, c, d)) return true;
    if (o4 == 0.0 && on_segment_exact(b, c, d)) return true;
    return false;
}

/// Removes ring indices whose vertex is exactly collinear with its two neighbours,
/// repeating until stable. Collinear outline vertices are permitted (MSH-002), so
/// they are dropped without producing a degenerate triangle.
void remove_collinear(std::vector<std::size_t>& ring, const std::vector<Vec2>& v)
{
    bool changed = true;
    while (changed && ring.size() > 3u) {
        changed = false;
        for (std::size_t i = 0; i < ring.size();) {
            const std::size_t m = ring.size();
            const Vec2 a = v[ring[(i + m - 1u) % m]];
            const Vec2 b = v[ring[i]];
            const Vec2 c = v[ring[(i + 1u) % m]];
            if (orient(a, b, c) == 0.0) {
                ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(i));
                changed = true;
                // Re-evaluate the new vertex now at index i; do not advance.
            } else {
                ++i;
            }
        }
    }
}

} // namespace

[[nodiscard]] Result<std::vector<Polygon>> triangulate_simple_polygon(const Polygon& polygon)
{
    const std::vector<Vec2>& v = polygon.vertices;
    const std::size_t n = v.size();

    // MSH-002: non-finite coordinates are an invalid argument.
    for (const Vec2 p : v) {
        if (!geom::is_finite(p)) {
            return std::unexpected(fail(ErrorCode::InvalidArgument, "non-finite vertex coordinate"));
        }
    }

    // MSH-002: fewer than three vertices.
    if (n < 3u) {
        return std::unexpected(fail(ErrorCode::InvalidMesh, "polygon has fewer than three vertices"));
    }

    // MSH-002: cyclic consecutive duplicate vertices, including equal first and last.
    for (std::size_t i = 0; i < n; ++i) {
        if (v[i] == v[(i + 1u) % n]) {
            return std::unexpected(fail(ErrorCode::InvalidMesh, "consecutive duplicate vertices"));
        }
    }

    // Signed area (shoelace), positive when counter-clockwise.
    const double area2 = geom::signed_area2(v);
    const double area = area2 * 0.5;

    // MSH-002: degenerate (collinear / zero) area.
    if (std::abs(area) <= geom::kEpsilonSquared) {
        return std::unexpected(fail(ErrorCode::InvalidMesh, "degenerate (zero) polygon area"));
    }

    // MSH-002: clockwise winding.
    if (area < 0.0) {
        return std::unexpected(fail(ErrorCode::InvalidMesh, "clockwise polygon winding"));
    }

    // MSH-002: self-intersections among non-adjacent edges. Starting j at i+2 skips
    // the adjacent edge i+1; the last edge (n-1) is adjacent to edge 0.
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = v[i];
        const Vec2 b = v[(i + 1u) % n];
        for (std::size_t j = i + 2u; j < n; ++j) {
            if (j == n - 1u && i == 0u) {
                continue;
            }
            if (segments_intersect(a, b, v[j], v[(j + 1u) % n])) {
                return std::unexpected(fail(ErrorCode::InvalidMesh, "self-intersecting polygon"));
            }
        }
    }

    // Work on original vertex indices so every output triangle corner is an input.
    std::vector<std::size_t> ring(n);
    for (std::size_t i = 0; i < n; ++i) {
        ring[i] = i;
    }
    remove_collinear(ring, v);

    // Deterministic ear clipping: scan the remaining ring in original cyclic order
    // and clip the first valid ear, repeating until a single triangle remains.
    std::vector<Polygon> out;
    while (ring.size() > 3u) {
        const std::size_t m = ring.size();
        bool clipped = false;
        for (std::size_t i = 0; i < m; ++i) {
            const std::size_t pi = ring[(i + m - 1u) % m];
            const std::size_t ci = ring[i];
            const std::size_t ni = ring[(i + 1u) % m];
            const Vec2 a = v[pi];
            const Vec2 b = v[ci];
            const Vec2 c = v[ni];

            // Reflex or (already removed) collinear vertices are not ears.
            if (orient(a, b, c) <= 0.0) {
                continue;
            }

            // The ear's triangle must contain no other remaining vertex.
            bool is_ear = true;
            for (std::size_t k = 0; k < m; ++k) {
                if (k == i || k == (i + m - 1u) % m || k == (i + 1u) % m) {
                    continue;
                }
                if (geom::point_in_triangle_ccw(v[ring[k]], a, b, c)) {
                    is_ear = false;
                    break;
                }
            }
            if (!is_ear) {
                continue;
            }

            out.push_back(Polygon{{a, b, c}});
            ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }

        // A simple, valid, positively-wound polygon always has an ear; fail safely
        // rather than loop forever on unforeseen input.
        if (!clipped) {
            return std::unexpected(fail(ErrorCode::InvalidMesh, "failed to triangulate polygon"));
        }
    }

    out.push_back(Polygon{{v[ring[0]], v[ring[1]], v[ring[2]]}});
    return out;
}

} // namespace vwmini
