// Mesh module: transactional validation of a triangle-only navigation mesh and
// point containment (MSH-004..MSH-009).
//
// Plan:
//   - `create` validates every triangle (finite, exactly-3 vertices, CCW and non-
//     degenerate), then rejects interior overlap, T-junctions, and non-manifold
//     edges with O(n^2) double-predicate comparisons before publishing an immutable
//     Impl; no partial mesh escapes.
//   - Adjacency (MSH-005) derives from complete shared edges whose endpoints match
//     within epsilon; vertex-only touches create no adjacency.
//   - `contains` (MSH-006) is const and allocation-free and applies the boundary
//     tolerance epsilon to closed edge segments.
//   - Pathfinding is intentionally not implemented in this module.

#include "mesh_internal.hpp"

#include "geometry_internal.hpp"

#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <ranges>
#include <utility>
#include <vector>

namespace vwmini {

namespace {

using geom::is_finite;
using geom::kEpsilon;
using geom::kEpsilonSquared;
using geom::orient;

Error fail(ErrorCode code, std::string message)
{
    return Error{code, std::move(message)};
}

/// Squared Euclidean distance between two points, in double.
[[nodiscard]] double dist2(Vec2 a, Vec2 b) noexcept
{
    const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
    const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
    return dx * dx + dy * dy;
}

/// Endpoints match within the shared-edge tolerance epsilon.
[[nodiscard]] bool same_point(Vec2 a, Vec2 b) noexcept
{
    return dist2(a, b) <= kEpsilonSquared;
}

/// Copies the three vertices of a validated triangle into a fixed-size array.
[[nodiscard]] std::array<Vec2, 3> triangle_vertices(const Polygon& tri) noexcept
{
    return {tri.vertices[0], tri.vertices[1], tri.vertices[2]};
}

/// Complete-edge match (MSH-005): endpoints correspond within epsilon, either order.
[[nodiscard]] bool edges_match(Vec2 a1, Vec2 b1, Vec2 a2, Vec2 b2) noexcept
{
    return (same_point(a1, a2) && same_point(b1, b2)) ||
           (same_point(a1, b2) && same_point(b1, a2));
}

/// True when closed segments [a,b] and [c,d] cross at a point strictly interior to
/// both (a proper crossing, excluding shared endpoints and collinear overlap).
[[nodiscard]] bool proper_crossing(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    const double o1 = orient(a, b, c);
    const double o2 = orient(a, b, d);
    const double o3 = orient(c, d, a);
    const double o4 = orient(c, d, b);
    return ((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) &&
           ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0));
}

/// True when p is strictly inside the CCW triangle (a, b, c), excluding boundary.
[[nodiscard]] bool point_strictly_in_ccw(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
{
    return orient(a, b, p) > 0.0 && orient(b, c, p) > 0.0 && orient(c, a, p) > 0.0;
}

/// True when v lies in the strict interior of closed segment [a,b]: exactly
/// collinear, not within epsilon of either endpoint (so it is a T-junction vertex,
/// not a shared vertex).
[[nodiscard]] bool vertex_in_edge_interior(Vec2 v, Vec2 a, Vec2 b) noexcept
{
    if (orient(a, b, v) != 0.0) {
        return false;
    }
    if (same_point(v, a) || same_point(v, b)) {
        return false;
    }
    const double vx = v.x, vy = v.y;
    return vx >= std::min(a.x, b.x) && vx <= std::max(a.x, b.x) &&
           vy >= std::min(a.y, b.y) && vy <= std::max(a.y, b.y);
}

/// True when the two CCW non-degenerate triangles overlap with positive interior
/// area. Covers proper edge crossings, containment of a vertex, and two triangles
/// sharing a complete edge on the same side (including duplicates).
[[nodiscard]] bool interiors_overlap(const Polygon& A, const Polygon& B) noexcept
{
    const std::array<Vec2, 3> a = triangle_vertices(A);
    const std::array<Vec2, 3> b = triangle_vertices(B);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (proper_crossing(a[i], a[(i + 1) % 3], b[j], b[(j + 1) % 3])) {
                return true;
            }
        }
    }
    for (int i = 0; i < 3; ++i) {
        if (point_strictly_in_ccw(a[i], b[0], b[1], b[2])) {
            return true;
        }
        if (point_strictly_in_ccw(b[i], a[0], a[1], a[2])) {
            return true;
        }
    }
    for (int i = 0; i < 3; ++i) {
        const Vec2 p = a[i];
        const Vec2 q = a[(i + 1) % 3];
        for (int j = 0; j < 3; ++j) {
            if (edges_match(p, q, b[j], b[(j + 1) % 3])) {
                // Shared complete edge. A's third vertex is always left of (p,q), so
                // B's third vertex s on the same side means the interiors overlap.
                if (orient(p, q, b[(j + 2) % 3]) > 0.0) {
                    return true;
                }
            }
        }
    }
    return false;
}

/// True when a vertex of one triangle lies in the interior of an edge of the other.
[[nodiscard]] bool has_t_junction(const Polygon& A, const Polygon& B) noexcept
{
    const std::array<Vec2, 3> a = triangle_vertices(A);
    const std::array<Vec2, 3> b = triangle_vertices(B);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (vertex_in_edge_interior(a[i], b[j], b[(j + 1) % 3])) {
                return true;
            }
            if (vertex_in_edge_interior(b[i], a[j], a[(j + 1) % 3])) {
                return true;
            }
        }
    }
    return false;
}

/// Euclidean distance from point p to the closed segment [a,b], in double.
[[nodiscard]] double point_segment_distance(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    const double px = p.x, py = p.y;
    const double ax = a.x, ay = a.y;
    const double abx = b.x - ax, aby = b.y - ay;
    const double apx = px - ax, apy = py - ay;
    const double len2 = abx * abx + aby * aby;
    if (len2 == 0.0) {
        return std::sqrt(apx * apx + apy * apy);
    }
    const double t = std::clamp((apx * abx + apy * aby) / len2, 0.0, 1.0);
    const double dx = px - (ax + t * abx);
    const double dy = py - (ay + t * aby);
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept
    : m_impl(std::move(impl))
{
}

[[nodiscard]] Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles)
{
    if (triangles.empty()) {
        return std::unexpected(fail(ErrorCode::InvalidMesh, "empty triangle list"));
    }
    const std::size_t n = triangles.size();

    // MSH-004 per-triangle validity: finiteness first (InvalidArgument), then shape.
    for (std::size_t i = 0; i < n; ++i) {
        const std::vector<Vec2>& v = triangles[i].vertices;
        for (const Vec2 p : v) {
            if (!is_finite(p)) {
                return std::unexpected(
                    fail(ErrorCode::InvalidArgument, "non-finite vertex coordinate"));
            }
        }
        if (v.size() != 3u) {
            return std::unexpected(fail(ErrorCode::InvalidMesh, "polygon is not a triangle"));
        }
        // Non-degenerate requires |area| > eps^2, i.e. |orient| > 2*eps^2; with CCW
        // this is orient > 2*eps^2. Anything at or below that is clockwise/degenerate.
        if (orient(v[0], v[1], v[2]) <= 2.0 * kEpsilonSquared) {
            return std::unexpected(
                fail(ErrorCode::InvalidMesh, "clockwise or degenerate triangle"));
        }
    }

    // MSH-004 pairwise topology: interior overlap and T-junctions.
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            if (interiors_overlap(triangles[i], triangles[j])) {
                return std::unexpected(
                    fail(ErrorCode::InvalidMesh, "overlapping triangle interiors"));
            }
            if (has_t_junction(triangles[i], triangles[j])) {
                return std::unexpected(
                    fail(ErrorCode::InvalidMesh, "T-junction vertex on an edge"));
            }
        }
    }

    // MSH-004 non-manifold edges and MSH-005 adjacency, in one O(n^2) pass. Each
    // complete edge may be shared by at most one other triangle.
    std::vector<std::array<std::size_t, 3>> neighbors(n);
    for (auto& nb : neighbors) {
        nb = {kNoNeighbor, kNoNeighbor, kNoNeighbor};
    }
    for (std::size_t i = 0; i < n; ++i) {
        const std::vector<Vec2>& v = triangles[i].vertices;
        for (int e = 0; e < 3; ++e) {
            const Vec2 p = v[e];
            const Vec2 q = v[(e + 1) % 3];
            std::size_t count = 0;
            std::size_t match = kNoNeighbor;
            for (std::size_t j = 0; j < n; ++j) {
                if (j == i) {
                    continue;
                }
                const std::vector<Vec2>& w = triangles[j].vertices;
                for (int f = 0; f < 3; ++f) {
                    if (edges_match(p, q, w[f], w[(f + 1) % 3])) {
                        ++count;
                        match = j;
                        break;
                    }
                }
            }
            if (count >= 2u) {
                return std::unexpected(fail(ErrorCode::InvalidMesh, "non-manifold edge"));
            }
            if (count == 1u) {
                neighbors[i][static_cast<std::size_t>(e)] = match;
            }
        }
    }

    // MSH-007/008: publish the immutable state only after all validation succeeds.
    auto impl = std::make_shared<const Impl>(std::move(triangles), std::move(neighbors));
    return NavMesh(std::move(impl));
}

[[nodiscard]] bool NavMesh::contains(Vec2 point) const noexcept
{
    if (!is_finite(point)) {
        return false;
    }
    for (const Polygon& tri : m_impl->triangles) {
        const Vec2 a = tri.vertices[0];
        const Vec2 b = tri.vertices[1];
        const Vec2 c = tri.vertices[2];
        // MSH-006: strictly inside, or within epsilon of any closed edge segment.
        if (point_strictly_in_ccw(point, a, b, c)) {
            return true;
        }
        if (point_segment_distance(point, a, b) <= kEpsilon ||
            point_segment_distance(point, b, c) <= kEpsilon ||
            point_segment_distance(point, c, a) <= kEpsilon) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] std::size_t NavMesh::cell_count() const noexcept
{
    return m_impl->triangles.size();
}

} // namespace vwmini
