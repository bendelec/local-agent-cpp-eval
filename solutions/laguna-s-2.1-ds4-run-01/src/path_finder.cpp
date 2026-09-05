// VWmini pathfinding implementation (WP4).
#include <vwmini/nav_mesh.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace vwmini {

inline constexpr float kEpsilon = 1e-4f;
inline constexpr float kEpsSq = kEpsilon * kEpsilon;

namespace {

float len_sq(Vec2 v) noexcept { return dot(v, v); }
float euclidean(Vec2 a, Vec2 b) noexcept { return length(a - b); }

enum class Orient { Pos, Neg, Zero };
Orient orient_sign(Vec2 a, Vec2 b, Vec2 c) noexcept
{
    const float cr = cross(b - a, c - a);
    if (cr > kEpsilon) {
        return Orient::Pos;
    }
    if (cr < -kEpsilon) {
        return Orient::Neg;
    }
    return Orient::Zero;
}

float dist_sq_point_seg(Vec2 p, Vec2 a, Vec2 b) noexcept
{
    const Vec2 ba = b - a;
    const float len2 = dot(ba, ba);
    if (len2 <= kEpsSq) {
        return dot(p - a, p - a);
    }
    float t = dot(p - a, ba) / len2;
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    const Vec2 closest = a + ba * t;
    return dot(p - closest, p - closest);
}

bool covered_by_triangle(Vec2 p, const Triangle& t) noexcept
{
    if (orient_sign(t.v[0], t.v[1], p) == Orient::Pos &&
        orient_sign(t.v[1], t.v[2], p) == Orient::Pos &&
        orient_sign(t.v[2], t.v[0], p) == Orient::Pos) {
        return true;
    }
    if (dist_sq_point_seg(p, t.v[0], t.v[1]) <= kEpsSq) {
        return true;
    }
    if (dist_sq_point_seg(p, t.v[1], t.v[2]) <= kEpsSq) {
        return true;
    }
    if (dist_sq_point_seg(p, t.v[2], t.v[0]) <= kEpsSq) {
        return true;
    }
    return false;
}

struct MeshView {
    const Triangle* tris{};
    std::size_t count{};

    bool contains_point(Vec2 p) const noexcept
    {
        for (std::size_t i = 0; i < count; ++i) {
            if (covered_by_triangle(p, tris[i])) {
                return true;
            }
        }
        return false;
    }
    std::size_t containing_index(Vec2 p) const noexcept
    {
        for (std::size_t i = 0; i < count; ++i) {
            if (covered_by_triangle(p, tris[i])) {
                return i;
            }
        }
        return static_cast<std::size_t>(-1);
    }
};

// True when the OPEN segment (a,b) lies entirely within the mesh union. Endpoints are
// assumed contained. Deterministic uniform sampling whose density scales with length.
bool segment_covered(const MeshView& mv, Vec2 a, Vec2 b) noexcept
{
    const float seg_len = euclidean(a, b);
    if (seg_len <= kEpsilon) {
        return true;
    }
    std::size_t steps = static_cast<std::size_t>(std::ceil(seg_len / (kEpsilon * 2.0f)));
    if (steps < 8) {
        steps = 8;
    }
    if (steps > 4096) {
        steps = 4096;
    }
    const Vec2 d = (b - a) * (1.0f / static_cast<float>(steps));
    for (std::size_t i = 1; i < steps; ++i) {
        const Vec2 mid = a + d * static_cast<float>(i);
        if (!mv.contains_point(mid)) {
            return false;
        }
    }
    return true;
}

} // namespace

Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal)
{
    if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
        !std::isfinite(goal.x) || !std::isfinite(goal.y)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite endpoint"});
    }

    const auto view = mesh.triangles_view();
    const MeshView mv{view.data(), view.size()};

    if (!mv.contains_point(start)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "start point is outside the navmesh"});
    }
    if (!mv.contains_point(goal)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "goal point is outside the navmesh"});
    }

    if (euclidean(start, goal) <= kEpsilon) {
        Path out{};
        out.points.push_back(start);
        return out;
    }

    // Direct visibility short-circuit: yields exactly [start, goal].
    if (segment_covered(mv, start, goal)) {
        Path out{};
        out.points.push_back(start);
        out.points.push_back(goal);
        return out;
    }

    const std::size_t sc = mv.containing_index(start);
    const std::size_t gc = mv.containing_index(goal);
    if (sc != gc) {
        return std::unexpected(Error{ErrorCode::NoPath, "endpoints lie in disconnected components"});
    }

    struct Node {
        Vec2 pos;
        std::uint32_t tri_idx;
    };
    std::vector<Node> nodes;
    nodes.push_back({start, static_cast<std::uint32_t>(sc)});
    nodes.push_back({goal, static_cast<std::uint32_t>(gc)});
    for (std::size_t ti = 0; ti < mv.count; ++ti) {
        const Triangle& t = mv.tris[ti];
        for (int v = 0; v < 3; ++v) {
            const Vec2 p = t.v[v];
            bool dup = false;
            for (const Node& n : nodes) {
                if (len_sq(n.pos - p) <= kEpsSq) {
                    dup = true;
                    break;
                }
            }
            if (!dup) {
                nodes.push_back({p, static_cast<std::uint32_t>(ti)});
            }
        }
    }

    const std::size_t n = nodes.size();
    std::vector<std::vector<std::pair<std::size_t, float>>> adj(n);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            if (nodes[i].tri_idx == nodes[j].tri_idx ||
                segment_covered(mv, nodes[i].pos, nodes[j].pos)) {
                const float w = euclidean(nodes[i].pos, nodes[j].pos);
                adj[i].push_back({j, w});
                adj[j].push_back({i, w});
            }
        }
    }

    // Deterministic Dijkstra: ascending-index tie-break on equal distance.
    std::vector<float> dist(n, std::numeric_limits<float>::infinity());
    std::vector<std::size_t> prev(n, static_cast<std::size_t>(-1));
    std::vector<bool> done(n, false);
    dist[0] = 0.0f;
    while (true) {
        std::size_t u = 0;
        float best = std::numeric_limits<float>::infinity();
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
        if (best == std::numeric_limits<float>::infinity()) {
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
            const float nd = dist[u] + e.second;
            if (nd < dist[e.first]) {
                dist[e.first] = nd;
                prev[e.first] = u;
            }
        }
    }

    if (dist[1] == std::numeric_limits<float>::infinity()) {
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
