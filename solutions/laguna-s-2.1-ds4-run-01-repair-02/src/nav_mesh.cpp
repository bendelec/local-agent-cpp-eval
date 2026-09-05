// VWmini navigation mesh implementation + pathfinding (opaque storage + queries).
#include <vwmini/nav_mesh.hpp>

// Translation-unit-private impl details consumed solely here. Must follow the public
// header which forward-declares NavMesh::Impl.
#include "navmesh_detail.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>
#include <vwmini/nav_mesh.hpp>

namespace vwmini {

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept : m_impl(std::move(impl)) {}

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles) {
    if (triangles.empty()) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "empty triangle list"});
    }

    std::vector<std::array<Vec2, 3>> tris;
    tris.reserve(triangles.size());
    for (const Polygon& poly : triangles) {
        if (poly.vertices.size() != 3) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "polygon is not a triangle"});
        }
        const Vec2 a = poly.vertices[0];
        const Vec2 b = poly.vertices[1];
        const Vec2 c = poly.vertices[2];
        if (!detail::finite_vec(a) || !detail::finite_vec(b) || !detail::finite_vec(c)) {
            return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite triangle vertex"});
        }
        // Degenerate iff |signed double area| <= epsilon^2 (strict CCW required).
        if (!detail::valid_triangle(a, b, c)) {
            return std::unexpected(
                Error{ErrorCode::InvalidMesh, "triangle is clockwise or degenerate"});
        }
        tris.push_back({a, b, c});
    }

    const std::size_t T = tris.size();

    // Pairwise geometry validation: shared-edge manifoldness, crossings, overlaps,
    // T-junctions, and non-manifold edges. Vertex-only touches are allowed & remain
    // non-adjacent (not fused into an edge).
    for (std::size_t i = 0; i < T; ++i) {
        for (std::size_t j = i + 1; j < T; ++j) {
            bool shared_edge_found = false;
            Vec2 se_a{}, se_b{}; // shared edge endpoints in i's orientation

            for (int e1 = 0; e1 < 3; ++e1) {
                const Vec2 a1 = tris[i][e1];
                const Vec2 b1 = tris[i][(e1 + 1) % 3];
                for (int e2 = 0; e2 < 3; ++e2) {
                    const Vec2 a2 = tris[j][e2];
                    const Vec2 b2 = tris[j][(e2 + 1) % 3];
                    if (detail::edges_match(a1, b1, a2, b2)) {
                        shared_edge_found = true;
                        se_a = a1;
                        se_b = b1;
                        continue;
                    }
                    const bool shares_endpoint =
                        detail::endpoints_near(a1, a2) || detail::endpoints_near(a1, b2) ||
                        detail::endpoints_near(b1, a2) || detail::endpoints_near(b1, b2);
                    if (!shares_endpoint && detail::proper_segment_intersection(a1, b1, a2, b2)) {
                        return std::unexpected(
                            Error{ErrorCode::InvalidMesh, "overlapping triangle interiors"});
                    }
                }
            }

            // T-junction detection: a vertex of one landing on a NON-endpoint interior
            // of an edge of the other.
            for (int k = 0; k < 3; ++k) {
                for (int e = 0; e < 3; ++e) {
                    const Vec2 a = tris[j][e];
                    const Vec2 b = tris[j][(e + 1) % 3];
                    if (detail::vertex_in_segment_interior(tris[i][k], a, b)) {
                        return std::unexpected(
                            Error{ErrorCode::InvalidMesh, "T-junction detected"});
                    }
                    if (detail::vertex_in_segment_interior(tris[j][k], tris[i][e],
                                                           tris[i][(e + 1) % 3])) {
                        return std::unexpected(
                            Error{ErrorCode::InvalidMesh, "T-junction detected"});
                    }
                }
            }

            if (shared_edge_found) {
                // Same-side apexes => two CCW triangles folding over a shared edge.
                const auto apex_of = [&](const std::array<Vec2, 3>& t) -> Vec2 {
                    for (int k = 0; k < 3; ++k) {
                        if (!detail::endpoints_near(t[k], se_a) &&
                            !detail::endpoints_near(t[k], se_b)) {
                            return t[k];
                        }
                    }
                    return t[0];
                };
                const Vec2 ai = apex_of(tris[i]);
                const Vec2 aj = apex_of(tris[j]);
                const double si = detail::signed_double_area(se_a, se_b, ai);
                const double sj = detail::signed_double_area(se_a, se_b, aj);
                if (si == 0.0 || sj == 0.0 || si * sj > 0.0) {
                    return std::unexpected(
                        Error{ErrorCode::InvalidMesh, "overlapping triangle interiors"});
                }
            } else {
                // No shared edge: overlap iff a vertex of one lies strictly inside the
                // other triangle. Coincident-vertex touch is not strict-inside.
                for (int k = 0; k < 3; ++k) {
                    if (detail::point_in_triangle_strict(tris[j][k], tris[i][0], tris[i][1],
                                                         tris[i][2])) {
                        return std::unexpected(
                            Error{ErrorCode::InvalidMesh, "overlapping triangle interiors"});
                    }
                }
                for (int k = 0; k < 3; ++k) {
                    if (detail::point_in_triangle_strict(tris[i][k], tris[j][0], tris[j][1],
                                                         tris[j][2])) {
                        return std::unexpected(
                            Error{ErrorCode::InvalidMesh, "overlapping triangle interiors"});
                    }
                }
            }
        }
    }

    // Build adjacency: each complete edge has exactly 0 or 1 partner (>1 => non-manifold).
    std::vector<std::array<std::size_t, 3>> neighbours(T);
    for (std::size_t i = 0; i < T; ++i) {
        for (int e = 0; e < 3; ++e) {
            neighbours[i][e] = Impl::no_cell;
        }
    }
    for (std::size_t i = 0; i < T; ++i) {
        for (int e = 0; e < 3; ++e) {
            if (neighbours[i][e] != Impl::no_cell) {
                continue;
            }
            const Vec2 a = tris[i][e];
            const Vec2 b = tris[i][(e + 1) % 3];
            std::size_t count = 0;
            std::size_t partner = Impl::no_cell;
            int partner_edge = 0;
            for (std::size_t j = 0; j < T; ++j) {
                if (j == i) {
                    continue;
                }
                for (int f = 0; f < 3; ++f) {
                    if (detail::edges_match(a, b, tris[j][f], tris[j][(f + 1) % 3])) {
                        ++count;
                        partner = j;
                        partner_edge = f;
                    }
                }
            }
            if (count > 1) {
                return std::unexpected(Error{ErrorCode::InvalidMesh, "non-manifold edge"});
            }
            if (count == 1) {
                neighbours[i][e] = partner;
                neighbours[partner][partner_edge] = i;
            }
        }
    }

    auto impl = std::make_shared<Impl>(std::move(tris), std::move(neighbours));
    return NavMesh(std::move(impl));
}

bool NavMesh::contains(Vec2 point) const noexcept {
    if (!m_impl || !detail::finite_vec(point)) {
        return false;
    }
    const auto& tris = m_impl->triangles;
    for (const auto& t : tris) {
        if (detail::contains_point_triangle(point, t[0], t[1], t[2])) {
            return true;
        }
    }
    return false;
}

std::size_t NavMesh::cell_count() const noexcept {
    return m_impl ? m_impl->triangles.size() : static_cast<std::size_t>(0);
}

// ---------------------------------------------------------------------------
// Pathfinding
// ---------------------------------------------------------------------------
namespace {

constexpr std::size_t NO_CELL = static_cast<std::size_t>(-1);

struct Portal {
    Vec2 left;
    Vec2 right;
};

// Shortest cell corridor from `start` to `goal` via Dijkstra over adjacency. Edge weight
// is Euclidean distance between cell centroids; ties break deterministically by smaller
// cell index so results are reproducible across runs/simulations.
std::vector<std::size_t> cell_corridor(const std::vector<std::array<Vec2, 3>>& triangles,
                                       const std::vector<std::array<std::size_t, 3>>& neighbours,
                                       std::size_t start, std::size_t goal) {
    const std::size_t T = triangles.size();
    std::vector<float> dist(T, std::numeric_limits<float>::infinity());
    std::vector<std::size_t> prev(T, NO_CELL);
    struct Item {
        float cost;
        std::size_t cell;
    };
    struct Cmp {
        bool operator()(const Item& a, const Item& b) const noexcept {
            return a.cost > b.cost || (a.cost == b.cost && a.cell > b.cell);
        }
    };
    std::vector<Item> heap;
    const auto push = [&](std::size_t cell, float cost) {
        heap.push_back({cost, cell});
        std::push_heap(heap.begin(), heap.end(), Cmp{});
    };
    const auto pop = [&]() -> Item {
        Item top = heap.front();
        std::pop_heap(heap.begin(), heap.end(), Cmp{});
        heap.pop_back();
        return top;
    };

    dist[start] = 0.0f;
    push(start, 0.0f);
    while (!heap.empty()) {
        const Item cur = pop();
        if (cur.cost > dist[cur.cell]) {
            continue;
        }
        if (cur.cell == goal) {
            break;
        }
        const auto& nbrs = neighbours[cur.cell];
        for (int e = 0; e < 3; ++e) {
            const std::size_t n = nbrs[e];
            if (n == NO_CELL) {
                continue;
            }
            const auto& ta = triangles[cur.cell];
            const auto& tb = triangles[n];
            const Vec2 cca{(ta[0].x + ta[1].x + ta[2].x) / 3.0f,
                           (ta[0].y + ta[1].y + ta[2].y) / 3.0f};
            const Vec2 ccb{(tb[0].x + tb[1].x + tb[2].x) / 3.0f,
                           (tb[0].y + tb[1].y + tb[2].y) / 3.0f};
            const float nd = cur.cost + length(ccb - cca);
            if (nd < dist[n]) {
                dist[n] = nd;
                prev[n] = cur.cell;
                push(n, nd);
            }
        }
    }

    std::vector<std::size_t> corridor;
    if (dist[goal] == std::numeric_limits<float>::infinity()) {
        return corridor;
    }
    for (std::size_t c = goal; c != NO_CELL; c = prev[c]) {
        corridor.push_back(c);
        if (c == start) {
            break;
        }
    }
    std::reverse(corridor.begin(), corridor.end());
    return corridor;
}

// Apex-based funnel with portal reprocessing. Each portal's `left`/`right` bound the
// corridor in its forward direction. A reflex corner that pushes a leg outside the cone
// walks the apex to that boundary vertex and re-processes the pinched portal, rather than
// drawing an unconditional straight segment out of the mesh. The goal is appended as a
// degenerate final portal {goal, goal}.
std::vector<Vec2> funnel_path(Vec2 start, Vec2 goal, const std::vector<Portal>& input) {
    std::vector<Portal> portals = input;
    portals.push_back(Portal{goal, goal});

    std::vector<Vec2> points;
    points.push_back(start);
    if (portals.size() == 1) {
        points.push_back(goal);
        return points;
    }

    Vec2 apex = start;
    Vec2 left = portals[0].left;
    Vec2 right = portals[0].right;
    std::size_t i = 1;
    while (i < portals.size()) {
        const Vec2 pL = portals[i].left;
        const Vec2 pR = portals[i].right;

        if (cross(left - apex, pR - apex) > 0.0f) {
            apex = left;
            points.push_back(left);
            left = pL;
            right = pR;
            continue;
        }
        if (cross(right - apex, pL - apex) < 0.0f) {
            apex = right;
            points.push_back(right);
            left = pL;
            right = pR;
            continue;
        }

        if (cross(pL - apex, left - apex) > 0.0f) {
            left = pL;
        }
        if (cross(pR - apex, right - apex) < 0.0f) {
            right = pR;
        }
        ++i;
    }

    if (points.back() != goal) {
        points.push_back(goal);
    }
    return points;
}

// Orient a shared edge so `left` lies on the left of `forward`.
Portal orient_portal(Vec2 a, Vec2 b, Vec2 forward) {
    const Vec2 mid = (a + b) * 0.5f;
    if (cross(forward, a - mid) >= cross(forward, b - mid)) {
        return Portal{a, b};
    }
    return Portal{b, a};
}

} // namespace

Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal) {
    if (!detail::finite_vec(start) || !detail::finite_vec(goal)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite path endpoint"});
    }
    if (!mesh.m_impl) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "empty navmesh"});
    }
    const NavMesh::Impl& m = *mesh.m_impl;

    // Validate both endpoints first: locate their containing cell (boundary-inclusive).
    std::size_t sc = NO_CELL;
    for (std::size_t i = 0; i < m.triangles.size(); ++i) {
        const auto& t = m.triangles[i];
        if (detail::contains_point_triangle(start, t[0], t[1], t[2])) {
            sc = i;
            break;
        }
    }
    if (sc == NO_CELL) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "start point is outside the navmesh"});
    }

    std::size_t gc = NO_CELL;
    for (std::size_t i = 0; i < m.triangles.size(); ++i) {
        const auto& t = m.triangles[i];
        if (detail::contains_point_triangle(goal, t[0], t[1], t[2])) {
            gc = i;
            break;
        }
    }
    if (gc == NO_CELL) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "goal point is outside the navmesh"});
    }

    // Exactly equal endpoints collapse to [start] under exact operator==. Distinct
    // endpoints within epsilon still require a normal route below.
    if (start == goal) {
        return Path{{start}};
    }

    // Same containing cell -> the straight segment stays inside that triangle.
    if (sc == gc) {
        return Path{{start, goal}};
    }

    std::vector<std::size_t> corridor = cell_corridor(m.triangles, m.neighbours, sc, gc);
    if (corridor.empty()) {
        return std::unexpected(
            Error{ErrorCode::NoPath, "endpoints lie in disconnected components"});
    }

    // Build oriented portals between consecutive corridor cells and funnel them.
    std::vector<Portal> portals;
    portals.reserve(corridor.size() - 1);
    for (std::size_t k = 0; k + 1 < corridor.size(); ++k) {
        const Vec2 forward =
            NavMesh::Impl::centroid(m, corridor[k + 1]) - NavMesh::Impl::centroid(m, corridor[k]);
        const auto edge = NavMesh::Impl::shared_edge(m, corridor[k], corridor[k + 1]);
        portals.push_back(orient_portal(edge[0], edge[1], forward));
    }

    std::vector<Vec2> raw = funnel_path(start, goal, portals);

    // Collapse redundant intermediate points closer than epsilon; keep exact endpoints.
    std::vector<Vec2> points;
    points.reserve(raw.size());
    for (std::size_t k = 0; k < raw.size(); ++k) {
        if (k == 0) {
            points.push_back(raw[k]);
            continue;
        }
        const Vec2 d = raw[k] - points.back();
        if (dot(d, d) <= kEpsSq && k != raw.size() - 1) {
            continue;
        }
        points.push_back(raw[k]);
    }
    if (points.empty() || points.front() != start) {
        points.insert(points.begin(), start);
    }
    if (points.size() < 2 || points.back() != goal) {
        points.push_back(goal);
    }
    return Path{std::move(points)};
}

} // namespace vwmini
