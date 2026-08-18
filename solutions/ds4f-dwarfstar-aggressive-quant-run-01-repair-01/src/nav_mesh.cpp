#include <vwmini/nav_mesh.hpp>

#include "internal.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <vector>

namespace vwmini {

namespace {

struct Tri {
    Vec2 v[3]; // counter-clockwise
};

struct Edge {
    int tri;
    Vec2 a;
    Vec2 b;
};

struct Neighbor {
    int idx; // neighbor triangle index
    Vec2 a;  // shared-edge endpoint 1
    Vec2 b;  // shared-edge endpoint 2
};

// Public-in-this-TU carrier for the mesh data, so free/anon helpers avoid the
// private NavMesh::Impl type. NavMesh::Impl wraps one MeshData value.
struct MeshData {
    std::vector<Tri> tris;
    std::vector<std::vector<Neighbor>> adj;
    std::vector<std::pair<Vec2, Vec2>> boundary;
};

} // anonymous namespace: internal types

// Private mesh representation (NavMesh::Impl forward-declared in the header).
struct NavMesh::Impl {
    MeshData data;
};

namespace {

// Two edges match when both are non-zero-length and their endpoints correspond
// within the geometric tolerance (MSH-005).
bool edgesMatch(const Edge& e1, const Edge& e2)
{
    const Vec2 d1 = e1.b - e1.a;
    const Vec2 d2 = e2.b - e2.a;
    if (length(d1) <= 0.0f || length(d2) <= 0.0f) {
        return false;
    }
    return (internal::distLe(e1.a, e2.a) && internal::distLe(e1.b, e2.b)) ||
           (internal::distLe(e1.a, e2.b) && internal::distLe(e1.b, e2.a));
}

// Number of edges geometrically matching edge[k] (always at least 1: itself).
int edgeMatchCount(const std::vector<Edge>& edges, std::size_t k)
{
    int matches = 1;
    for (std::size_t j = 0; j < edges.size(); ++j) {
        if (j != k && edgesMatch(edges[k], edges[j])) {
            ++matches;
        }
    }
    return matches;
}

// Sign of the oriented area of (a,b,p): +1 above the line a-b, -1 below,
// 0 within kEpsilon of it (distance tolerance).
int sideOfLine(Vec2 a, Vec2 b, Vec2 p)
{
    const float v = internal::triArea2(a, b, p);
    const float len = length(b - a);
    if (v > internal::kEpsilon * len) {
        return 1;
    }
    if (v < -internal::kEpsilon * len) {
        return -1;
    }
    return 0;
}

// True when two triangles overlap in their interiors (MSH-004).
bool trianglesOverlap(const Tri& t1, const Tri& t2)
{
    // A vertex strictly inside the other triangle.
    for (int i = 0; i < 3; ++i) {
        if (internal::pointInTri(t1.v[i], t2.v[0], t2.v[1], t2.v[2])) {
            return true;
        }
        if (internal::pointInTri(t2.v[i], t1.v[0], t1.v[1], t1.v[2])) {
            return true;
        }
    }
    // An edge properly crossing an edge of the other triangle.
    for (int i = 0; i < 3; ++i) {
        const Vec2 a1 = t1.v[i];
        const Vec2 b1 = t1.v[(i + 1) % 3];
        for (int j = 0; j < 3; ++j) {
            const Vec2 a2 = t2.v[j];
            const Vec2 b2 = t2.v[(j + 1) % 3];
            if (internal::properIntersect(a1, b1, a2, b2)) {
                return true;
            }
        }
    }
    // Collinear edge overlap with the two triangles' interiors on the same side
    // of the shared edge line (e.g. two identical triangles): the interiors
    // overlap even though no vertex is strictly inside and no edge properly
    // crosses. Valid adjacent triangles put their third vertices on opposite
    // sides of the shared edge, so they are not rejected here.
    for (int i = 0; i < 3; ++i) {
        const Vec2 a1 = t1.v[i];
        const Vec2 b1 = t1.v[(i + 1) % 3];
        const Vec2 c1 = t1.v[(i + 2) % 3];
        for (int j = 0; j < 3; ++j) {
            const Vec2 a2 = t2.v[j];
            const Vec2 b2 = t2.v[(j + 1) % 3];
            const Vec2 c2 = t2.v[(j + 2) % 3];
            if (!internal::collinearOverlap(a1, b1, a2, b2)) {
                continue;
            }
            const int s1 = sideOfLine(a1, b1, c1);
            const int s2 = sideOfLine(a1, b1, c2);
            if (s1 != -s2 || s1 == 0 || s2 == 0) {
                return true;
            }
        }
    }
    return false;
}

int cellOf(const MeshData& impl, Vec2 p)
{
    for (std::size_t i = 0; i < impl.tris.size(); ++i) {
        const Tri& t = impl.tris[i];
        if (internal::pointInTri(p, t.v[0], t.v[1], t.v[2])) {
            return static_cast<int>(i);
        }
    }
    for (std::size_t i = 0; i < impl.tris.size(); ++i) {
        const Tri& t = impl.tris[i];
        for (int e = 0; e < 3; ++e) {
            if (internal::distPointSegment(p, t.v[e], t.v[(e + 1) % 3]) <= internal::kEpsilon) {
                return static_cast<int>(i);
            }
        }
    }
    return -1;
}

bool reachable(const MeshData& impl, int s, int g)
{
    std::vector<bool> visited(impl.tris.size(), false);
    std::vector<int> stack;
    stack.push_back(s);
    visited[static_cast<std::size_t>(s)] = true;
    while (!stack.empty()) {
        const int t = stack.back();
        stack.pop_back();
        if (t == g) {
            return true;
        }
        for (const Neighbor& nb : impl.adj[static_cast<std::size_t>(t)]) {
            if (!visited[static_cast<std::size_t>(nb.idx)]) {
                visited[static_cast<std::size_t>(nb.idx)] = true;
                stack.push_back(nb.idx);
            }
        }
    }
    return false;
}

// Direct segment is contained iff every point is inside a triangle or within
// kEpsilon of a triangle edge (MSH-006 / SIM-002). Dense sampling replaces the
// earlier boundary-crossing test, which wrongly rejected segments that cross a
// boundary edge within the tolerance zone (e.g. endpoints on the boundary).
bool directContained(const MeshData& impl, Vec2 s, Vec2 g)
{
    auto contained = [&](Vec2 q) { return cellOf(impl, q) != -1; };
    return internal::segmentContainedBy(s, g, contained);
}

std::optional<std::vector<int>> astar(const MeshData& impl, int s, int g)
{
    const std::size_t n = impl.tris.size();
    std::vector<Vec2> centroid(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Tri& t = impl.tris[i];
        centroid[i] = (t.v[0] + t.v[1] + t.v[2]) * (1.0f / 3.0f);
    }

    std::vector<float> gs(n, std::numeric_limits<float>::infinity());
    std::vector<float> fs(n, std::numeric_limits<float>::infinity());
    std::vector<int> parent(n, -1);
    std::vector<bool> closed(n, false);

    auto heur = [&](int t) {
        return length(centroid[static_cast<std::size_t>(t)] - centroid[static_cast<std::size_t>(g)]);
    };

    struct Cmp {
        const std::vector<float>& fs;
        bool operator()(int a, int b) const
        {
            const float fa = fs[static_cast<std::size_t>(a)];
            const float fb = fs[static_cast<std::size_t>(b)];
            if (fa != fb) {
                return fa > fb; // smaller f first
            }
            return a > b; // deterministic tie-break on triangle index
        }
    };

    std::priority_queue<int, std::vector<int>, Cmp> open{Cmp{fs}};
    gs[static_cast<std::size_t>(s)] = 0.0f;
    fs[static_cast<std::size_t>(s)] = heur(s);
    open.push(s);

    while (!open.empty()) {
        const int t = open.top();
        open.pop();
        if (closed[static_cast<std::size_t>(t)]) {
            continue;
        }
        closed[static_cast<std::size_t>(t)] = true;
        if (t == g) {
            break;
        }
        for (const Neighbor& nb : impl.adj[static_cast<std::size_t>(t)]) {
            if (closed[static_cast<std::size_t>(nb.idx)]) {
                continue;
            }
            const float cost =
                length(centroid[static_cast<std::size_t>(t)] - centroid[static_cast<std::size_t>(nb.idx)]);
            const float ng = gs[static_cast<std::size_t>(t)] + cost;
            if (ng < gs[static_cast<std::size_t>(nb.idx)]) {
                gs[static_cast<std::size_t>(nb.idx)] = ng;
                fs[static_cast<std::size_t>(nb.idx)] = ng + heur(nb.idx);
                parent[static_cast<std::size_t>(nb.idx)] = t;
                open.push(nb.idx);
            }
        }
    }

    if (!closed[static_cast<std::size_t>(g)]) {
        return std::nullopt;
    }

    std::vector<int> path;
    int cur = g;
    while (cur != -1) {
        path.push_back(cur);
        if (cur == s) {
            break;
        }
        cur = parent[static_cast<std::size_t>(cur)];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Exact shortest contained polyline through a corridor of triangles.
// Builds the visibility graph over the corridor's vertices plus start/goal and
// runs Dijkstra. The corridor union is a simple region, so the visibility-graph
// shortest path equals the true shortest path through the corridor (SIM-003).
std::optional<std::vector<Vec2>> corridorPath(Vec2 start, Vec2 goal, const std::vector<int>& corridor,
                                              const MeshData& impl)
{
    // Nodes: start (0), goal (1), then corridor triangle vertices (deduplicated).
    std::vector<Vec2> nodes;
    nodes.push_back(start);
    nodes.push_back(goal);
    for (const int t : corridor) {
        for (int i = 0; i < 3; ++i) {
            const Vec2 v = impl.tris[static_cast<std::size_t>(t)].v[i];
            bool dup = false;
            for (const Vec2& n : nodes) {
                if (internal::distLe(v, n)) {
                    dup = true;
                    break;
                }
            }
            if (!dup) {
                nodes.push_back(v);
            }
        }
    }
    const std::size_t N = nodes.size();

    // Corridor boundary edges: corridor-triangle edges shared by exactly one
    // corridor triangle. Properly crossing one exits the corridor.
    std::vector<Edge> allEdges;
    for (const int t : corridor) {
        for (int e = 0; e < 3; ++e) {
            const Vec2 a = impl.tris[static_cast<std::size_t>(t)].v[e];
            const Vec2 b = impl.tris[static_cast<std::size_t>(t)].v[(e + 1) % 3];
            allEdges.push_back({t, a, b});
        }
    }
    std::vector<std::pair<Vec2, Vec2>> bounds;
    for (std::size_t k = 0; k < allEdges.size(); ++k) {
        if (edgeMatchCount(allEdges, k) == 1) {
            bounds.push_back({allEdges[k].a, allEdges[k].b});
        }
    }

    // Visibility adjacency matrix. A visibility edge must stay inside the mesh
    // (dense containment sampling catches collinear/tangential exits into
    // non-walkable space that proper crossing misses, e.g. across an L-notch)
    // and must not properly cross a corridor boundary edge.
    std::vector<std::vector<bool>> vis(N, std::vector<bool>(N, false));
    auto contained = [&](Vec2 q) { return cellOf(impl, q) != -1; };
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = i + 1; j < N; ++j) {
            bool visible = internal::segmentContainedBy(nodes[i], nodes[j], contained);
            if (visible) {
                for (const auto& b : bounds) {
                    if (internal::properIntersect(nodes[i], nodes[j], b.first, b.second)) {
                        visible = false;
                        break;
                    }
                }
            }
            vis[i][j] = vis[j][i] = visible;
        }
    }

    // Dijkstra over the visibility graph (start=0, goal=1).
    std::vector<float> dist(N, std::numeric_limits<float>::infinity());
    std::vector<int> parent(N, -1);
    struct Cmp {
        const std::vector<float>& dist;
        bool operator()(std::size_t a, std::size_t b) const
        {
            const float da = dist[a];
            const float db = dist[b];
            if (da != db) {
                return da > db; // smaller distance first
            }
            return a > b; // deterministic tie-break on node index
        }
    };
    std::priority_queue<std::size_t, std::vector<std::size_t>, Cmp> open{Cmp{dist}};
    dist[0] = 0.0f;
    open.push(0);
    while (!open.empty()) {
        const std::size_t u = open.top();
        open.pop();
        if (dist[u] == std::numeric_limits<float>::infinity()) {
            continue;
        }
        if (u == 1) {
            break;
        }
        for (std::size_t v = 0; v < N; ++v) {
            if (!vis[u][v]) {
                continue;
            }
            const float w = length(nodes[u] - nodes[v]);
            const float nd = dist[u] + w;
            if (nd < dist[v]) {
                dist[v] = nd;
                parent[v] = static_cast<int>(u);
                open.push(v);
            }
        }
    }
    if (dist[1] == std::numeric_limits<float>::infinity()) {
        return std::nullopt;
    }

    std::vector<Vec2> pts;
    pts.push_back(start);
    std::vector<std::size_t> chain;
    std::size_t cur = 1;
    while (cur != 0) {
        chain.push_back(cur);
        cur = static_cast<std::size_t>(parent[cur]);
    }
    std::reverse(chain.begin(), chain.end());
    for (const std::size_t c : chain) {
        pts.push_back(nodes[c]);
    }
    pts.push_back(goal);
    return pts;
}

std::vector<Vec2> dedupIntermediate(std::vector<Vec2> pts)
{
    if (pts.size() <= 2) {
        return pts;
    }
    std::vector<Vec2> out;
    out.push_back(pts.front());
    for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
        if (length(pts[i] - out.back()) >= internal::kEpsilon) {
            out.push_back(pts[i]);
        }
    }
    out.push_back(pts.back());
    return out;
}

} // namespace

NavMesh::NavMesh(std::shared_ptr<const Impl> impl) noexcept : m_impl(std::move(impl))
{
}

Result<NavMesh> NavMesh::create(std::vector<Polygon> triangles)
{
    if (triangles.empty()) {
        return std::unexpected(Error{ErrorCode::InvalidMesh, "empty triangle list"});
    }

    std::vector<Tri> tris;
    tris.reserve(triangles.size());
    for (const Polygon& p : triangles) {
        if (p.vertices.size() != 3) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "non-triangle cell"});
        }
        for (const Vec2& v : p.vertices) {
            if (!internal::finite(v)) {
                return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite vertex"});
            }
        }
        const Vec2 a = p.vertices[0];
        const Vec2 b = p.vertices[1];
        const Vec2 c = p.vertices[2];
        if (internal::triArea2(a, b, c) <= internal::kEpsilon * internal::kEpsilon) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "clockwise or degenerate triangle"});
        }
        tris.push_back({a, b, c});
    }

    // Reject triangles whose interiors overlap (MSH-004).
    for (std::size_t i = 0; i < tris.size(); ++i) {
        for (std::size_t j = i + 1; j < tris.size(); ++j) {
            if (trianglesOverlap(tris[i], tris[j])) {
                return std::unexpected(Error{ErrorCode::InvalidMesh, "overlapping triangle interiors"});
            }
        }
    }

    std::vector<Edge> edges;
    edges.reserve(tris.size() * 3);
    for (std::size_t i = 0; i < tris.size(); ++i) {
        for (int e = 0; e < 3; ++e) {
            edges.push_back({static_cast<int>(i), tris[i].v[e], tris[i].v[(e + 1) % 3]});
        }
    }

    // Non-manifold: an edge appearing in more than two triangles (MSH-004).
    for (std::size_t k = 0; k < edges.size(); ++k) {
        if (edgeMatchCount(edges, k) > 2) {
            return std::unexpected(Error{ErrorCode::InvalidMesh, "non-manifold edge"});
        }
    }

    std::vector<std::vector<Neighbor>> adj(tris.size());
    for (std::size_t k = 0; k < edges.size(); ++k) {
        for (std::size_t j = k + 1; j < edges.size(); ++j) {
            if (edgesMatch(edges[k], edges[j]) && edges[k].tri != edges[j].tri) {
                const int t1 = edges[k].tri;
                const int t2 = edges[j].tri;
                const Vec2 a = edges[k].a;
                const Vec2 b = edges[k].b;
                adj[static_cast<std::size_t>(t1)].push_back({t2, a, b});
                adj[static_cast<std::size_t>(t2)].push_back({t1, a, b});
            }
        }
    }

    // Boundary edges: those appearing in exactly one triangle.
    std::vector<std::pair<Vec2, Vec2>> boundary;
    for (std::size_t k = 0; k < edges.size(); ++k) {
        if (edgeMatchCount(edges, k) == 1) {
            boundary.push_back({edges[k].a, edges[k].b});
        }
    }

    // T-junction: a vertex lying in the middle of another triangle's edge (MSH-004).
    for (std::size_t k = 0; k < edges.size(); ++k) {
        const Vec2 a = edges[k].a;
        const Vec2 b = edges[k].b;
        for (const Tri& t : tris) {
            for (int v = 0; v < 3; ++v) {
                const Vec2 p = t.v[v];
                if (internal::distLe(p, a) || internal::distLe(p, b)) {
                    continue;
                }
                if (internal::distPointSegment(p, a, b) <= internal::kEpsilon) {
                    return std::unexpected(Error{ErrorCode::InvalidMesh, "T-junction vertex on edge"});
                }
            }
        }
    }

    auto impl = std::make_shared<const Impl>(Impl{MeshData{tris, adj, boundary}});
    return NavMesh(std::move(impl));
}

bool NavMesh::contains(Vec2 point) const noexcept
{
    if (!internal::finite(point)) {
        return false;
    }
    return cellOf(m_impl->data, point) != -1;
}

std::size_t NavMesh::cell_count() const noexcept
{
    return m_impl->data.tris.size();
}

Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal)
{
    if (!internal::finite(start) || !internal::finite(goal)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite endpoint"});
    }
    if (start == goal) {
        return Path{{start}};
    }

    const MeshData& impl = mesh.m_impl->data;
    if (!mesh.contains(start)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "start outside mesh"});
    }
    if (!mesh.contains(goal)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "goal outside mesh"});
    }

    const int s = cellOf(impl, start);
    const int g = cellOf(impl, goal);
    if (s != g && !reachable(impl, s, g)) {
        return std::unexpected(Error{ErrorCode::NoPath, "disconnected endpoints"});
    }

    if (directContained(impl, start, goal)) {
        return Path{{start, goal}};
    }

    auto corridor = astar(impl, s, g);
    if (!corridor.has_value()) {
        return std::unexpected(Error{ErrorCode::NoPath, "no connecting corridor"});
    }

    auto ptsOpt = corridorPath(start, goal, corridor.value(), impl);
    if (!ptsOpt.has_value()) {
        return std::unexpected(Error{ErrorCode::NoPath, "no visible path through corridor"});
    }
    std::vector<Vec2> pts = dedupIntermediate(std::move(ptsOpt.value()));
    return Path{std::move(pts)};
}

} // namespace vwmini
