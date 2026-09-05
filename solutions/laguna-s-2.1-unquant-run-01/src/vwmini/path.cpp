#include <vwmini/nav_mesh.hpp>

#include <vwmini/detail.hpp>
#include <vwmini/navmesh_detail.hpp>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

namespace vwmini {

namespace {

constexpr std::size_t NO_CELL = static_cast<std::size_t>(-1);

struct Portal {
    Vec2 left;
    Vec2 right;
};

// Shortest cell corridor from `start` to `goal` via Dijkstra over adjacency.
// Edge weight is Euclidean distance between cell centroids. Ties are broken
// deterministically by smaller cell index.
std::vector<std::size_t> cell_corridor(
    const std::vector<std::array<Vec2, 3>>& triangles,
    const std::vector<std::array<std::size_t, 3>>& neighbours, std::size_t start,
    std::size_t goal)
{
    const std::size_t T = triangles.size();
    std::vector<float> dist(T, std::numeric_limits<float>::infinity());
    std::vector<std::size_t> prev(T, NO_CELL);
    // Priority queue ordering: smallest cost wins; on ties smallest cell index.
    struct Item {
        float cost;
        std::size_t cell;
    };
    struct Cmp {
        bool operator()(const Item& a, const Item& b) const noexcept
        {
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
            const Vec2 ca{(ta[0].x + ta[1].x + ta[2].x) / 3.0f,
                          (ta[0].y + ta[1].y + ta[2].y) / 3.0f};
            const Vec2 cb{(tb[0].x + tb[1].x + tb[2].x) / 3.0f,
                          (tb[0].y + tb[1].y + tb[2].y) / 3.0f};
            const float nd = cur.cost + length(cb - ca);
            if (nd < dist[n]) {
                dist[n] = nd;
                prev[n] = cur.cell;
                push(n, nd);
            }
        }
    }

    std::vector<std::size_t> corridor;
    if (dist[goal] == std::numeric_limits<float>::infinity()) {
        return corridor; // unreachable
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

// Apex-based funnel with portal reprocessing. `portals` are ordered start->goal,
// each portal's `left` is the corridor's left-boundary endpoint, `right` the
// right-boundary endpoint for that shared edge. On a pinch the apex walks to the
// boundary vertex and the pinched portal is re-processed against the new cone.
std::vector<Vec2> funnel_path(Vec2 start, Vec2 goal,
                              const std::vector<Portal>& portals)
{
    std::vector<Vec2> points;
    points.push_back(start);
    if (portals.empty()) {
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

        // pR beyond the left leg (pR is left of apex->left): apex walks to left.
        if (cross(left - apex, pR - apex) > 0.0f) {
            apex = left;
            points.push_back(left);
            left = pL;
            right = pR;
            continue; // reprocess this portal against the new cone
        }
        // pL beyond the right leg (pL is right of apex->right): apex walks to right.
        if (cross(right - apex, pL - apex) < 0.0f) {
            apex = right;
            points.push_back(right);
            right = pR;
            left = pL;
            continue; // reprocess this portal against the new cone
        }

        // Tighten each leg with the more extreme boundary point of this portal.
        if (cross(left - apex, pL - apex) > 0.0f) {
            left = pL;
        }
        if (cross(right - apex, pR - apex) < 0.0f) {
            right = pR;
        }
        ++i;
    }

    points.push_back(goal);
    return points;
}

// Orient a shared edge so `left` is on the left of the forward direction.
Portal orient_portal(Vec2 a, Vec2 b, Vec2 forward)
{
    const Vec2 mid = (a + b) * 0.5f;
    const float ca = cross(forward, a - mid);
    const float cb = cross(forward, b - mid);
    if (ca >= cb) {
        return Portal{a, b};
    }
    return Portal{b, a};
}

} // namespace

Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal)
{
    if (!detail::finite(start) || !detail::finite(goal)) {
        return std::unexpected(
            Error{ErrorCode::InvalidArgument, "non-finite path endpoint"});
    }

    const NavMesh::Impl& m = *mesh.m_impl;

    // Locate the triangle that contains each endpoint (boundary-inclusive).
    std::size_t sc = NO_CELL;
    for (std::size_t i = 0; i < m.triangles.size(); ++i) {
        const auto& t = m.triangles[i];
        if (detail::contains_point_triangle(start, t[0], t[1], t[2])) {
            sc = i;
            break;
        }
    }
    if (sc == NO_CELL) {
        return std::unexpected(
            Error{ErrorCode::OutsideMesh, "start point is outside the mesh"});
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
        return std::unexpected(
            Error{ErrorCode::OutsideMesh, "goal point is outside the mesh"});
    }

    // Exactly equal endpoints -> single point.
    if (start == goal) {
        return Path{{start}};
    }

    // Same containing cell -> straight segment stays within that triangle.
    if (sc == gc) {
        return Path{{start, goal}};
    }

    std::vector<std::size_t> corridor = cell_corridor(m.triangles, m.neighbours, sc, gc);
    if (corridor.empty()) {
        return std::unexpected(Error{ErrorCode::NoPath, "no connecting route"});
    }

    // Build portals between consecutive corridor cells.
    std::vector<Portal> portals;
    portals.reserve(corridor.size() - 1);
    for (std::size_t i = 0; i + 1 < corridor.size(); ++i) {
        const Vec2 forward =
            NavMesh::Impl::centroid(m, corridor[i + 1]) -
            NavMesh::Impl::centroid(m, corridor[i]);
        const auto edge =
            NavMesh::Impl::shared_edge(m, corridor[i], corridor[i + 1]);
        portals.push_back(orient_portal(edge[0], edge[1], forward));
    }

    std::vector<Vec2> raw = funnel_path(start, goal, portals);

    // Collapse consecutive intermediate points closer than epsilon.
    std::vector<Vec2> points;
    points.reserve(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i) {
        if (i == 0) {
            points.push_back(raw[i]);
            continue;
        }
        const Vec2& last = points.back();
        const Vec2 d = raw[i] - last;
        if (dot(d, d) <= detail::epsilon_sq && i != raw.size() - 1) {
            continue; // omit redundant intermediate point
        }
        points.push_back(raw[i]);
    }
    // Guarantee start and goal are the exact endpoints.
    if (points.empty() || points.front() != start) {
        points.insert(points.begin(), start);
    }
    if (points.size() < 2 || points.back() != goal) {
        points.push_back(goal);
    }
    return Path{std::move(points)};
}

} // namespace vwmini
