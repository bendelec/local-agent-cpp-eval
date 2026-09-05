// VWmini navigation mesh implementation + pathfinding (opaque storage + queries).
#include <vwmini/nav_mesh.hpp>

// Translation-unit-private impl details consumed solely here. Must follow the public
// header which forward-declares NavMesh::Impl.
#include "navmesh_detail.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>
#include <vwmini/nav_mesh.hpp>

namespace vwmini {

using detail::containing_index;
using detail::covered_by_closed_triangle;
using detail::edge_improper_contact;
using detail::endpoints_near;
using detail::finite_vec;
using detail::Orient;
using detail::orient_strict;
using detail::segment_covered;
using detail::signed_double_area;
using detail::Tri;

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept : m_impl(std::move(impl)) {}

// ---------------------------------------------------------------------------
// Pairwise triangle relationship classification
// ---------------------------------------------------------------------------
namespace {

struct EdgeMatch {
    int edge_i = -1; // matching edge index in tri i
    int edge_j = -1; // matching edge index in tri j
};

// Find a complete shared edge between two triangles: both endpoints of one edge of ti
// coincide (within epsilon) with both endpoints of an edge of tj, traversed either same
// or opposite direction. Returns encoded match or {-1,-1}.
EdgeMatch shared_edge(const Tri& ti, const Tri& tj) noexcept {
    for (int e = 0; e < 3; ++e) {
        const Vec2 ai = ti.v[e];
        const Vec2 bi = ti.v[(e + 1) % 3];
        if (!detail::endpoints_near(ai, bi)) {
            continue; // degenerate edge in input (shouldn't happen post-validation)
        }
        for (int f = 0; f < 3; ++f) {
            const Vec2 aj = tj.v[f];
            const Vec2 bj = tj.v[(f + 1) % 3];
            const bool same = detail::endpoints_near(ai, aj) && detail::endpoints_near(bi, bj);
            const bool rev = detail::endpoints_near(ai, bj) && detail::endpoints_near(bi, aj);
            if (same || rev) {
                return {e, f};
            }
        }
    }
    return {-1, -1};
}

enum class PairCode { SharedEdge, Disjoint, Invalid };
struct PairInfo {
    PairCode code{PairCode::Disjoint};
    int edge_i{-1};
    int edge_j{-1};
};

PairInfo classify_pair(const Tri& ti, const Tri& tj) noexcept {
    PairInfo info{};
    const EdgeMatch m = shared_edge(ti, tj);
    if (m.edge_i >= 0) {
        // Verify apexes lie on opposite sides of the shared line => manifold, no overlap.
        const Vec2 ap = ti.v[m.edge_i];
        const Vec2 bp = ti.v[(m.edge_i + 1) % 3];
        const Vec2 opp_i = ti.v[(m.edge_i + 2) % 3];
        const Vec2 opp_j = tj.v[(m.edge_j + 2) % 3];
        const auto si = orient_strict(ap, bp, opp_i);
        const auto sj = orient_strict(ap, bp, opp_j);
        if (si != sj && si != Orient::Zero && sj != Orient::Zero) {
            info.code = PairCode::SharedEdge;
            info.edge_i = m.edge_i;
            info.edge_j = m.edge_j;
        } else {
            info.code = PairCode::Invalid; // duplicate/folded/overlapping along that edge
        }
        return info;
    }

    // No full shared edge: reject improper contact while allowing isolated vertex touches.
    for (const Vec2 v : ti.v) {
        if (detail::strictly_inside(v, tj.v)) {
            info.code = PairCode::Invalid;
            return info;
        }
    }
    for (const Vec2 v : tj.v) {
        if (detail::strictly_inside(v, ti.v)) {
            info.code = PairCode::Invalid;
            return info;
        }
    }

    // Transverse crossings / collinear overlaps / T-junctions are improper unless they
    // occur only at coincident endpoints (vertex-only touch is allowed & non-adjacent).
    for (int e = 0; e < 3; ++e) {
        const Vec2 ae = ti.v[e];
        const Vec2 be = ti.v[(e + 1) % 3];
        for (int f = 0; f < 3; ++f) {
            const Vec2 cf = tj.v[f];
            const Vec2 df = tj.v[(f + 1) % 3];
            if (edge_improper_contact(ae, be, cf, df)) {
                info.code = PairCode::Invalid;
                return info;
            }
        }
    }

    info.code = PairCode::Disjoint;
    return info;
}

void link_adjacency(std::vector<std::array<int, 3>>& adj, int i, int j, int ei, int ej) noexcept {
    adj[i][ei] = j;
    adj[j][ej] = i;
}

} // namespace

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles) {
    auto impl = std::make_shared<Impl>();
    impl->tris.reserve(triangles.size());
    impl->adj.reserve(triangles.size());

    for (const Polygon& poly : triangles) {
        if (poly.vertices.size() != 3) {
            return std::unexpected(
                Error{ErrorCode::InvalidMesh, "triangle must have exactly three vertices"});
        }
        const Vec2 a = poly.vertices[0];
        const Vec2 b = poly.vertices[1];
        const Vec2 c = poly.vertices[2];
        if (!finite_vec(a) || !finite_vec(b) || !finite_vec(c)) {
            return std::unexpected(
                Error{ErrorCode::InvalidArgument, "non-finite vertex coordinate"});
        }
        // Degenerate iff |signed double area| <= epsilon^2 (double-precision comparison).
        if (!(signed_double_area(a, b, c) > kEpsSq)) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "degenerate or non-CCW triangle"});
        }
        impl->tris.push_back(Tri{a, b, c});
        impl->adj.push_back({-1, -1, -1});
    }

    if (impl->tris.empty()) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "empty triangle list"});
    }

    const int n = static_cast<int>(impl->tris.size());
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const PairInfo pi = classify_pair(impl->tris[i], impl->tris[j]);
            switch (pi.code) {
            case PairCode::SharedEdge:
                link_adjacency(impl->adj, i, j, pi.edge_i, pi.edge_j);
                break;
            case PairCode::Disjoint:
                continue;
            default:
                return std::unexpected(
                    Error{ErrorCode::InvalidMesh, "overlapping or non-manifold triangles"});
            }
        }
    }

    return NavMesh(std::move(impl));
}

bool NavMesh::contains(Vec2 point) const noexcept {
    if (!m_impl || !finite_vec(point)) {
        return false;
    }
    for (const Tri& t : m_impl->tris) {
        if (covered_by_closed_triangle(point, t.v)) {
            return true;
        }
    }
    return false;
}

std::size_t NavMesh::cell_count() const noexcept {
    return m_impl ? m_impl->tris.size() : 0;
}

// ---------------------------------------------------------------------------
// Pathfinding
// ---------------------------------------------------------------------------
namespace {

// Reachability over the complete-edge adjacency graph from `start_tri` to `goal_tri`.
bool connected(const std::vector<std::array<int, 3>>& adj, std::size_t start_tri,
               std::size_t goal_tri) noexcept {
    const int n = static_cast<int>(adj.size());
    if (n == 0) {
        return false;
    }
    if (start_tri == goal_tri) {
        return true;
    }
    std::vector<bool> seen(n, false);
    std::queue<int> q;
    q.push(static_cast<int>(start_tri));
    seen[start_tri] = true;
    while (!q.empty()) {
        const int u = q.front();
        q.pop();
        for (int e = 0; e < 3; ++e) {
            const int v = adj[u][e];
            if (v >= 0 && !seen[v]) {
                if (static_cast<std::size_t>(v) == goal_tri) {
                    return true;
                }
                seen[v] = true;
                q.push(v);
            }
        }
    }
    return false;
}

struct Node {
    Vec2 pos;
    std::uint32_t tri_idx;
};

} // namespace

Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal) {
    if (!finite_vec(start) || !finite_vec(goal)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite endpoint"});
    }
    if (!mesh.m_impl) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "empty navmesh"});
    }
    const auto& tris = mesh.m_impl->tris;

    // Exact caller-supplied endpoints collapse to a single-point path.
    if (endpoints_near(start, goal)) {
        Path out{};
        out.points.push_back(start);
        return out;
    }

    // Both endpoints must lie within the closed mesh before routing.
    const std::size_t sc0 = containing_index(tris, start);
    const std::size_t gc0 = containing_index(tris, goal);
    if (sc0 == static_cast<std::size_t>(-1)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "start point is outside the navmesh"});
    }
    if (gc0 == static_cast<std::size_t>(-1)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "goal point is outside the navmesh"});
    }

    // Direct visibility short-circuit: yields exactly [start, goal].
    if (segment_covered(tris, start, goal)) {
        Path out{};
        out.points.push_back(start);
        out.points.push_back(goal);
        return out;
    }

    // Connectivity via the accepted complete-edge adjacency graph. Disconnected => NoPath.
    const std::size_t sc = containing_index(tris, start);
    const std::size_t gc = containing_index(tris, goal);
    if (sc == static_cast<std::size_t>(-1) || gc == static_cast<std::size_t>(-1) ||
        !connected(mesh.m_impl->adj, sc, gc)) {
        return std::unexpected(
            Error{ErrorCode::NoPath, "endpoints lie in disconnected components"});
    }

    // Visibility graph over {start, goal} plus distinct mesh vertices.
    std::vector<Node> nodes;
    nodes.push_back({start, static_cast<std::uint32_t>(sc)});
    nodes.push_back({goal, static_cast<std::uint32_t>(gc)});
    for (std::size_t ti = 0; ti < tris.size(); ++ti) {
        const Tri& t = tris[ti];
        for (int v = 0; v < 3; ++v) {
            bool dup = false;
            for (const Node& nd : nodes) {
                if (endpoints_near(nd.pos, t.v[v])) {
                    dup = true;
                    break;
                }
            }
            if (!dup) {
                nodes.push_back({t.v[v], static_cast<std::uint32_t>(ti)});
            }
        }
    }

    const std::size_t n = nodes.size();
    std::vector<std::vector<std::pair<std::size_t, double>>> adj(n);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            if (nodes[i].tri_idx == nodes[j].tri_idx ||
                segment_covered(tris, nodes[i].pos, nodes[j].pos)) {
                const double w = std::hypot(static_cast<double>(nodes[i].pos.x - nodes[j].pos.x),
                                            static_cast<double>(nodes[i].pos.y - nodes[j].pos.y));
                adj[i].push_back({j, w});
                adj[j].push_back({i, w});
            }
        }
    }

    // Deterministic Dijkstra with ascending-index tie-break on equal distance.
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<std::size_t> prev(n, static_cast<std::size_t>(-1));
    std::vector<bool> done(n, false);
    dist[0] = 0.0;
    while (true) {
        std::size_t u = 0;
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < n; ++i) {
            if (done[i]) {
                continue;
            }
            if (dist[i] < best) {
                best = dist[i];
                u = i;
            } else if (dist[i] == best && i < u) {
                u = i;
            }
        }
        if (best == std::numeric_limits<double>::infinity()) {
            break;
        }
        done[u] = true;
        if (u == 1) {
            break;
        }
        for (const auto& e : adj[u]) {
            if (done[e.first]) {
                continue;
            }
            const double nd = dist[u] + e.second;
            if (nd < dist[e.first]) {
                dist[e.first] = nd;
                prev[e.first] = u;
            }
        }
    }

    if (dist[1] == std::numeric_limits<double>::infinity()) {
        return std::unexpected(Error{ErrorCode::NoPath, "no contained route between endpoints"});
    }

    std::vector<Vec2> chain;
    for (std::size_t cur = 1; cur != 0; cur = prev[cur]) {
        chain.push_back(nodes[cur].pos);
        if (prev[cur] == cur) {
            break;
        }
    }
    chain.push_back(start);
    std::reverse(chain.begin(), chain.end());

    Path out{};
    out.points = std::move(chain);
    return out;
}

} // namespace vwmini
