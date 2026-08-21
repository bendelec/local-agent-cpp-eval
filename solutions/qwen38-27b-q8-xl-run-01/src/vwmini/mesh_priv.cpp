#include "vwmini/mesh_priv.hpp"

#include <algorithm>

namespace vwmini::detail {

namespace {

[[nodiscard]] bool near_eps(Vec2 p, Vec2 q) noexcept
{
    return squared_distance(p, q) <= epsilon2;
}

// MSH-005: two complete edges match when corresponding endpoints are no
// farther than `epsilon`.
[[nodiscard]] bool edges_match(Vec2 a, Vec2 b, Vec2 c, Vec2 d) noexcept
{
    return (near_eps(a, c) && near_eps(b, d)) || (near_eps(a, d) && near_eps(b, c));
}

// True when p lies strictly inside the CCW triangle (a, b, c); delegates to
// the shared detail predicate.
[[nodiscard]] bool strictly_inside_triangle(Vec2 p, const TriCell& t) noexcept
{
    return strictly_inside(p, t.a, t.b, t.c);
}

// True when `v` lies in the open part of segment [a, b], i.e. on the segment
// and farther than `epsilon` from both endpoints (MSH-004 T-junction test).
[[nodiscard]] bool on_open_segment(Vec2 v, Vec2 a, Vec2 b)
{
    return point_on_segment_exact(v, a, b) && squared_distance(v, a) > epsilon2 &&
           squared_distance(v, b) > epsilon2;
}

// True when a vertex of one triangle lies in the open interior of an edge of
// the other: a T-junction (MSH-004).
[[nodiscard]] bool has_t_junction(const TriCell& t, const TriCell& u)
{
    const Vec2 tv[3] = {t.a, t.b, t.c};
    const Vec2 uv[3] = {u.a, u.b, u.c};
    for (const Vec2 v : tv)
        for (int k = 0; k < 3; ++k)
            if (on_open_segment(v, uv[k], uv[(k + 1) % 3]))
                return true;
    for (const Vec2 v : uv)
        for (int k = 0; k < 3; ++k)
            if (on_open_segment(v, tv[k], tv[(k + 1) % 3]))
                return true;
    return false;
}

// True when two complete edges cross in their interiors, or lie on one line
// with a non-zero overlap that is not a full shared-edge match (MSH-004:
// overlapping triangle interiors / improper edge contact).
[[nodiscard]] bool edges_improperly_meet(Vec2 a, Vec2 b, Vec2 c, Vec2 d)
{
    if (segments_properly_intersect(a, b, c, d))
        return true;
    return segments_collinear_overlap(a, b, c, d) && !edges_match(a, b, c, d);
}

// True when any complete edge of `t` improperly meets any complete edge of `u`.
[[nodiscard]] bool edges_improperly_cross(const TriCell& t, const TriCell& u)
{
    const Vec2 tv[3] = {t.a, t.b, t.c};
    const Vec2 uv[3] = {u.a, u.b, u.c};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (edges_improperly_meet(tv[i], tv[(i + 1) % 3], uv[j], uv[(j + 1) % 3]))
                return true;
    return false;
}

// True when a vertex of one triangle is strictly inside the other's interior:
// the interiors overlap (MSH-004).
[[nodiscard]] bool vertex_strictly_inside(const TriCell& t, const TriCell& u)
{
    if (strictly_inside_triangle(t.a, u) || strictly_inside_triangle(t.b, u) ||
        strictly_inside_triangle(t.c, u))
        return true;
    return strictly_inside_triangle(u.a, t) || strictly_inside_triangle(u.b, t) ||
           strictly_inside_triangle(u.c, t);
}

// True when the two triangles coincide (all vertices of each lie on the
// boundary of the other, within `epsilon`): a duplicated triangle, whose
// interiors fully overlap (MSH-004).
[[nodiscard]] bool triangles_coincident(const TriCell& t, const TriCell& u)
{
    auto all_on_boundary = [](const TriCell& first, const TriCell& other) {
        const Vec2 tv[3] = {first.a, first.b, first.c};
        const Vec2 ov[3] = {other.a, other.b, other.c};
        for (const Vec2 p : tv)
        {
            bool on = false;
            for (int k = 0; k < 3 && !on; ++k)
                on = point_on_segment_exact(p, ov[k], ov[(k + 1) % 3]) ||
                     near_eps(p, ov[k]) || near_eps(p, ov[(k + 1) % 3]);
            if (!on)
                return false;
        }
        return true;
    };
    return all_on_boundary(t, u) && all_on_boundary(u, t);
}

std::optional<Error> mesh_validation_error(std::vector<TriCell>& cells)
{
    // Pairwise interior / T-junction / duplicate checks (MSH-004).
    for (std::size_t i = 0; i < cells.size(); ++i)
        for (std::size_t j = i + 1; j < cells.size(); ++j)
        {
            if (edges_improperly_cross(cells[i], cells[j]) ||
                vertex_strictly_inside(cells[i], cells[j]))
                return Error{ErrorCode::InvalidMesh, "overlapping triangle interiors"};
            if (has_t_junction(cells[i], cells[j]))
                return Error{ErrorCode::InvalidMesh, "T-junction in mesh"};
            if (triangles_coincident(cells[i], cells[j]))
                return Error{ErrorCode::InvalidMesh, "duplicate triangle"};
        }

    // Non-manifold edge: a complete edge matched (within epsilon) by three or
    // more triangles (MSH-004). Overlap checks catch most geometric cases;
    // this keeps the rule explicit and guards epsilon-matched near-duplicates.
    for (std::size_t i = 0; i < cells.size(); ++i)
    {
        const Vec2 ei[3] = {cells[i].a, cells[i].b, cells[i].c};
        for (int k = 0; k < 3; ++k)
        {
            const Vec2 p = ei[k];
            const Vec2 q = ei[(k + 1) % 3];
            std::size_t matchers = 0;
            for (std::size_t j = 0; j < cells.size() && matchers < 2; ++j)
            {
                if (j == i)
                    continue;
                const Vec2 ej[3] = {cells[j].a, cells[j].b, cells[j].c};
                for (int m = 0; m < 3; ++m)
                    if (edges_match(p, q, ej[m], ej[(m + 1) % 3]))
                    {
                        ++matchers;
                        break;
                    }
            }
            if (matchers >= 2)
                return Error{ErrorCode::InvalidMesh, "non-manifold edge"};
        }
    }

    // Shared-edge adjacency (MSH-005).
    for (std::size_t i = 0; i < cells.size(); ++i)
        for (std::size_t j = i + 1; j < cells.size(); ++j)
        {
            const Vec2 ei[3] = {cells[i].a, cells[i].b, cells[i].c};
            const Vec2 ej[3] = {cells[j].a, cells[j].b, cells[j].c};
            for (int k = 0; k < 3; ++k)
                for (int m = 0; m < 3; ++m)
                    if (edges_match(ei[k], ei[(k + 1) % 3], ej[m], ej[(m + 1) % 3]))
                    {
                        cells[i].neighbors.emplace_back(
                            j, std::array{ei[k], ei[(k + 1) % 3]});
                        cells[j].neighbors.emplace_back(
                            i, std::array{ej[m], ej[(m + 1) % 3]});
                        break;
                    }
        }

    return std::nullopt;
}

} // namespace

Result<MeshImpl> build_mesh_impl(const std::vector<Polygon>& triangles)
{
    if (triangles.empty())
        return fail(ErrorCode::InvalidMesh, "empty triangle list");

    std::vector<TriCell> cells(triangles.size());
    for (std::size_t i = 0; i < triangles.size(); ++i)
    {
        const Polygon& poly = triangles[i];
        if (poly.vertices.size() != 3)
            return fail(ErrorCode::InvalidMesh, "polygon is not a triangle");
        const Vec2 v[3] = {poly.vertices[0], poly.vertices[1], poly.vertices[2]};
        for (const Vec2& p : v)
            if (!is_finite(p))
                return fail(ErrorCode::InvalidArgument, "non-finite triangle vertex");
        // CCW and non-degenerate: signed double area must exceed epsilon^2.
        if (triangle_area2(v[0], v[1], v[2]) <= epsilon2)
            return fail(ErrorCode::InvalidMesh, "clockwise or degenerate triangle");
        cells[i] = TriCell{v[0], v[1], v[2], {}};
    }

    if (auto err = mesh_validation_error(cells))
        return std::unexpected(*err);
    return MeshImpl{std::move(cells)};
}

bool cell_contains(const TriCell& cell, Vec2 point) noexcept
{
    if (strictly_inside_triangle(point, cell))
        return true;
    const Vec2 v[3] = {cell.a, cell.b, cell.c};
    for (int k = 0; k < 3; ++k)
        if (point_segment_distance(point, v[k], v[(k + 1) % 3]) <= epsilon)
            return true;
    return false;
}

bool mesh_contains(const MeshImpl& impl, Vec2 point)
{
    if (!is_finite(point))
        return false;
    for (const TriCell& cell : impl.cells)
        if (cell_contains(cell, point))
            return true;
    return false;
}

} // namespace vwmini::detail
