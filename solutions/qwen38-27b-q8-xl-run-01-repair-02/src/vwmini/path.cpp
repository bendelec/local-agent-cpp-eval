// Path finding (SIM-001..SIM-004).
//
// `find_path`:
//   1. validates endpoints (InvalidArgument / OutsideMesh, SIM-001);
//   2. returns the exact direct segment when it is contained (SIM-001);
//   3. otherwise routes over the cell-adjacency graph with Dijkstra, then
//      string-pulls the corridor of shared-edge portals (funnel-equivalent
//      shortening, SIM-003);
//   4. verifies every output segment with the exact MSH-006 containment test
//      and reports NoPath when nothing verifiable is found (SIM-002 gate).
//
// Determinism (SIM-004): fixed candidate order, a (distance, cell-index)
// heap for Dijkstra, and pure float arithmetic with fixed iteration order.
#include "vwmini/path.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <queue>
#include <vector>

namespace vwmini::detail {

namespace {

// Largest total uncovered length (metres) still treated as covered; absorbs
// float rounding in the interval arithmetic below.
constexpr float kCoverageGapTolerance = 1e-7f;
// Two endpoints closer than this are treated as one point.
constexpr float kDegenerateSegmentLength2 = 1e-16f;
// String-pulling settle threshold (metres) and sweep cap.
constexpr float kPullSettle = 1e-9f;
constexpr int kPullMaxSweeps = 64;
// First-order optimality tolerance for the pulled route (metres).
constexpr float kPullVerifyTolerance = 1e-6f;
// Pulled points within this distance of a portal endpoint snap to it (float
// intersection noise; far below the containment tolerance).
constexpr float kSnapDistance2 = 1e-10f; // (1e-5 m)^2
// sin(theta) below this counts as collinear in output simplification.
constexpr float kCollinearSin = 1e-6f;

struct TInterval {
    bool empty = true;
    float lo{};
    float hi{};
};

[[nodiscard]] TInterval make_interval(float lo, float hi)
{
    return lo > hi ? TInterval{} : TInterval{false, lo, hi};
}

[[nodiscard]] TInterval intersect(const TInterval& a, const TInterval& b)
{
    if (a.empty || b.empty)
        return {};
    return make_interval(std::max(a.lo, b.lo), std::min(a.hi, b.hi));
}

// {t in [0,1] : A + t*B >= 0}.
[[nodiscard]] TInterval half_ge(float A, float B)
{
    if (B == 0.f)
        return A >= 0.f ? TInterval{false, 0.f, 1.f} : TInterval{};
    const float t = -A / B;
    return B > 0.f ? make_interval(std::max(0.f, t), 1.f)
                   : make_interval(0.f, std::min(1.f, t));
}

// {t in [0,1] : A + t*B <= 0}.
[[nodiscard]] TInterval half_le(float A, float B)
{
    if (B == 0.f)
        return A <= 0.f ? TInterval{false, 0.f, 1.f} : TInterval{};
    const float t = -A / B;
    return B > 0.f ? make_interval(0.f, std::min(1.f, t))
                   : make_interval(std::max(0.f, t), 1.f);
}

// {t in [0,1] : c2*t^2 + c1*t + c0 <= 0}, with c2 > 0.
[[nodiscard]] TInterval quadratic_le(float c2, float c1, float c0)
{
    const float disc = c1 * c1 - 4.f * c2 * c0;
    if (disc < 0.f)
        return {}; // c2 > 0: the quadratic is positive everywhere
    const float sq = std::sqrt(disc);
    float r1, r2;
    if (c1 == 0.f)
    {
        r1 = -sq / (2.f * c2);
        r2 = sq / (2.f * c2);
    }
    else
    {
        // Numerically stable root pair: q = -1/2 (c1 + sign(c1) sq).
        const float q = -0.5f * (c1 + (c1 > 0.f ? sq : -sq));
        const float ra = q / c2;
        const float rb = c0 / q;
        r1 = std::min(ra, rb);
        r2 = std::max(ra, rb);
    }
    return make_interval(std::max(0.f, r1), std::min(1.f, r2));
}

// Appends the set of t in [0,1] for which |S(t) - seg([p, q])| <= epsilon,
// where S(t) = a + t*d. The tolerance band of a segment is a capsule: three
// regions split by the projection of S(t) onto the edge line.
void edge_capsule_intervals(Vec2 a, Vec2 d, Vec2 p, Vec2 q, std::vector<TInterval>& out)
{
    const Vec2 u = q - p;
    const float l2 = dot(u, u);
    const float d2 = dot(d, d);
    const float e = epsilon;
    const Vec2 w = a - p;
    const float A = dot(w, u); // projection(t) = A + t*B (times |u|, unnormalized)
    const float B = dot(d, u);
    const float C = cross(u, w); // lateral(t) = C + t*D (times |u|, unnormalized)
    const float D = cross(u, d);

    // Projection at or before p: distance to p.
    TInterval r1 = half_le(A, B);
    r1 = intersect(r1, quadratic_le(d2, 2.f * dot(w, d), dot(w, w) - e * e));
    out.push_back(r1);

    // Projection onto the edge: distance to the supporting line.
    const float h_max = e * std::sqrt(l2);
    TInterval r3 = intersect(half_ge(A, B), half_le(A - l2, B));
    r3 = intersect(r3, half_le(C - h_max, D));
    r3 = intersect(r3, half_le(-C - h_max, -D));
    out.push_back(r3);

    // Projection at or after q: distance to q.
    const Vec2 w2 = w - u;
    TInterval r2 = half_ge(A - l2, B);
    r2 = intersect(r2, quadratic_le(d2, 2.f * dot(w2, d), dot(w2, w2) - e * e));
    out.push_back(r2);
}

// {t in [0,1] : S(t) is in the strict interior of the CCW triangle `cell`}.
[[nodiscard]] TInterval triangle_interior_interval(Vec2 a, Vec2 d, const TriCell& cell)
{
    const Vec2 v[3] = {cell.a, cell.b, cell.c};
    TInterval r{false, 0.f, 1.f};
    for (int k = 0; k < 3; ++k)
    {
        const Vec2 p = v[k];
        const Vec2 u = v[(k + 1) % 3] - p;
        r = intersect(r, half_ge(cross(u, a - p), cross(u, d)));
        if (r.empty)
            return r;
    }
    return r;
}

} // namespace

bool segment_contained(const MeshImpl& mesh, Vec2 a, Vec2 b)
{
    if (!is_finite(a) || !is_finite(b))
        return false;
    const float d2 = squared_distance(a, b);
    if (d2 <= kDegenerateSegmentLength2)
        return mesh_contains(mesh, a) && mesh_contains(mesh, b);
    const Vec2 d = b - a;

    std::vector<TInterval> intervals;
    intervals.reserve(mesh.cells.size() * 4);
    for (const TriCell& cell : mesh.cells)
    {
        intervals.push_back(triangle_interior_interval(a, d, cell));
        const Vec2 v[3] = {cell.a, cell.b, cell.c};
        for (int k = 0; k < 3; ++k)
            edge_capsule_intervals(a, d, v[k], v[(k + 1) % 3], intervals);
    }

    // Merge the intervals (sorted by lo) and measure the covered length of
    // [0, 1]; all intervals are clipped to [0, 1] on construction.
    std::sort(intervals.begin(), intervals.end(),
              [](const TInterval& x, const TInterval& y) { return x.lo < y.lo; });
    float covered = 0.f;
    float lo = 0.f, hi = 0.f;
    bool active = false;
    for (const TInterval& iv : intervals)
    {
        if (iv.empty)
            continue;
        if (!active)
        {
            active = true;
            lo = iv.lo;
            hi = iv.hi;
        }
        else if (iv.lo <= hi)
        {
            hi = std::max(hi, iv.hi);
        }
        else
        {
            covered += hi - lo;
            lo = iv.lo;
            hi = iv.hi;
        }
    }
    if (active)
        covered += hi - lo;
    return (1.f - covered) * std::sqrt(d2) <= kCoverageGapTolerance;
}

namespace {

// The shortest cell route from `start` to `goal` (both inclusive), empty when
// the cells are disconnected. The heap orders (distance, cell index), so the
// result is deterministic for identical input.
std::vector<std::size_t> dijkstra_cell_route(const MeshImpl& mesh, std::size_t start,
                                             std::size_t goal)
{
    const std::size_t n = mesh.cells.size();
    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> dist(n, inf);
    std::vector<int> prev(n, -1);
    std::vector<char> done(n, 0);
    using HeapValue = std::pair<float, std::size_t>; // (distance, cell index)
    std::priority_queue<HeapValue, std::vector<HeapValue>, std::greater<>> pq;
    dist[start] = 0.f;
    pq.push({0.f, start});
    while (!pq.empty())
    {
        const auto [d, i] = pq.top();
        pq.pop();
        if (done[i])
            continue;
        done[i] = 1;
        if (i == goal)
            break;
        for (const auto& [j, edge] : mesh.cells[i].neighbors)
        {
            if (done[j])
                continue;
            const float nd = d + length(edge[0] - edge[1]);
            if (nd < dist[j])
            {
                dist[j] = nd;
                prev[j] = static_cast<int>(i);
                pq.push({nd, j});
            }
        }
    }
    if (!done[goal])
        return {};
    std::vector<std::size_t> route;
    for (std::size_t c = goal;; c = static_cast<std::size_t>(prev[c]))
    {
        route.push_back(c);
        if (c == start)
            break;
    }
    std::reverse(route.begin(), route.end());
    return route;
}

// The shared complete edge of adjacent cells `a` and `b`, oriented in `a`'s
// own CCW direction (so `a` lies on its left).
std::array<Vec2, 2> shared_portal(const MeshImpl& mesh, std::size_t a, std::size_t b)
{
    for (const auto& [j, edge] : mesh.cells[a].neighbors)
        if (j == b)
            return edge;
    return {mesh.cells[a].a, mesh.cells[a].a}; // unreachable for valid routes
}

// The point of the segment [e0, e1] hit by the segment A->B, if any.
std::optional<Vec2> segment_crossing(Vec2 A, Vec2 B, Vec2 e0, Vec2 e1)
{
    const Vec2 d = B - A;
    const Vec2 e = e1 - e0;
    const float den = cross(d, e);
    if (den == 0.f)
        return std::nullopt; // parallel or collinear: keep the current point
    const float t = cross(e0 - A, e) / den;
    const float s = cross(e0 - A, d) / den;
    if (t < 0.f || t > 1.f || s < 0.f || s > 1.f)
        return std::nullopt;
    return A + d * t;
}

// Shortest polyline from `start` to `goal` crossing each portal segment in
// order (string-pulling / funnel-equivalent, SIM-003). Every intermediate
// point stays on its portal, so with consecutive portals sharing a convex
// cell the polyline stays inside the corridor. Returns empty when the
// iteration does not settle (fail-safe; the caller then reports NoPath).
std::vector<Vec2> string_pull(Vec2 start, Vec2 goal,
                              const std::vector<std::array<Vec2, 2>>& portals)
{
    const std::size_t m = portals.size();
    if (m == 0)
        return {start, goal};

    auto block_cost = [](Vec2 A, Vec2 X, Vec2 B) {
        return length(A - X) + length(X - B);
    };

    std::vector<Vec2> pts(2 + m);
    pts.front() = start;
    pts.back() = goal;
    for (std::size_t i = 0; i < m; ++i)
        pts[1 + i] = (portals[i][0] + portals[i][1]) * 0.5f;

    for (int sweep = 0; sweep < kPullMaxSweeps; ++sweep)
    {
        bool changed = false;
        for (std::size_t i = 0; i < m; ++i)
        {
            const std::size_t idx = 1 + i;
            const Vec2 A = pts[idx - 1];
            const Vec2 B = pts[idx + 1];
            const Vec2 e0 = portals[i][0];
            const Vec2 e1 = portals[i][1];
            Vec2 best = pts[idx];
            float best_cost = block_cost(A, best, B);
            for (const Vec2 cand : {e0, e1})
            {
                const float c = block_cost(A, cand, B);
                if (c < best_cost)
                {
                    best_cost = c;
                    best = cand;
                }
            }
            if (const auto x = segment_crossing(A, B, e0, e1))
            {
                const float c = block_cost(A, *x, B);
                if (c < best_cost)
                {
                    best_cost = c;
                    best = *x;
                }
            }
            if (squared_distance(best, pts[idx]) > kPullSettle * kPullSettle)
            {
                pts[idx] = best;
                changed = true;
            }
        }
        if (!changed)
            break;
    }

    // First-order check: the objective is jointly convex in the portal
    // crossing points, so coordinate-wise optimality on each portal is a
    // global minimum. Fail-safe when the sweep cap was hit.
    for (std::size_t i = 0; i < m; ++i)
    {
        const std::size_t idx = 1 + i;
        const Vec2 A = pts[idx - 1];
        const Vec2 B = pts[idx + 1];
        const Vec2 X = pts[idx];
        const float c = block_cost(A, X, B);
        const float c0 = block_cost(A, portals[i][0], B);
        const float c1 = block_cost(A, portals[i][1], B);
        float best_alternative = std::min(c0, c1);
        if (const auto x = segment_crossing(A, B, portals[i][0], portals[i][1]))
            best_alternative = std::min(best_alternative, block_cost(A, *x, B));
        if (c > best_alternative + kPullVerifyTolerance)
            return {};
    }

    // Snap pulled points that landed within kSnapDistance2 of a portal
    // endpoint onto the exact endpoint: the intersection computation carries
    // float noise, and exact vertices make the collinear simplification below
    // exact. Containment is unaffected (the deviation is far below epsilon).
    for (std::size_t i = 0; i < m; ++i)
    {
        for (const Vec2 endpoint : {portals[i][0], portals[i][1]})
            if (squared_distance(pts[1 + i], endpoint) <= kSnapDistance2)
            {
                pts[1 + i] = endpoint;
                break;
            }
    }

    // SIM-002: omit intermediate points closer than epsilon to a neighbour or
    // collinear with both neighbours. Endpoints are always preserved.
    bool changed = true;
    while (changed && pts.size() > 2)
    {
        changed = false;
        for (std::size_t i = 1; i + 1 < pts.size(); ++i)
        {
            const Vec2 prev = pts[i - 1];
            const Vec2 next = pts[i + 1];
            const float l1 = length(pts[i] - prev);
            const float l2 = length(next - pts[i]);
            const float sin_theta =
                std::abs(cross(pts[i] - prev, next - pts[i])) / std::max(1.f, l1 * l2);
            if (l1 <= epsilon || l2 <= epsilon || sin_theta <= kCollinearSin)
            {
                pts.erase(pts.begin() + static_cast<std::ptrdiff_t>(i));
                changed = true;
                break;
            }
        }
    }
    return pts;
}

[[nodiscard]] float polyline_length(const std::vector<Vec2>& points)
{
    float total = 0.f;
    for (std::size_t i = 1; i < points.size(); ++i)
        total += length(points[i] - points[i - 1]);
    return total;
}

// SIM-002 gate: every consecutive segment of the route is contained under
// MSH-006, including the endpoints.
bool route_segments_contained(const MeshImpl& mesh, const std::vector<Vec2>& route)
{
    if (route.empty())
        return false;
    for (std::size_t i = 1; i < route.size(); ++i)
        if (!segment_contained(mesh, route[i - 1], route[i]))
            return false;
    return true;
}

} // namespace

} // namespace vwmini::detail

namespace vwmini {

Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal)
{
    // SIM-001 validation.
    if (!detail::is_finite(start) || !detail::is_finite(goal))
        return detail::fail(ErrorCode::InvalidArgument, "path endpoints must be finite");
    const auto& impl = mesh.m_impl->data;
    if (!detail::mesh_contains(impl, start))
        return detail::fail(ErrorCode::OutsideMesh, "start point is outside the mesh");
    if (!detail::mesh_contains(impl, goal))
        return detail::fail(ErrorCode::OutsideMesh, "goal point is outside the mesh");
    if (start == goal)
        return Path{std::vector<Vec2>{start}};

    // SIM-001: the straight segment is contained -> exactly [start, goal].
    if (detail::segment_contained(impl, start, goal))
        return Path{std::vector<Vec2>{start, goal}};

    // Candidate cells: the endpoint is contained by the mesh, so at least one
    // cell qualifies (strict interior or epsilon edge band). Points on shared
    // boundaries are contained by several cells; trying each in index order
    // keeps the selection deterministic.
    auto containing_cells = [&](Vec2 p) {
        std::vector<std::size_t> out;
        for (std::size_t i = 0; i < impl.cells.size(); ++i)
            if (detail::cell_contains(impl.cells[i], p))
                out.push_back(i);
        return out;
    };
    const std::vector<std::size_t> start_cells = containing_cells(start);
    const std::vector<std::size_t> goal_cells = containing_cells(goal);

    Path best;
    bool found = false;
    float best_length = std::numeric_limits<float>::infinity();
    for (const std::size_t s : start_cells)
    {
        for (const std::size_t g : goal_cells)
        {
            if (s == g)
                continue; // one convex cell: the straight segment is contained
            const std::vector<std::size_t> route = detail::dijkstra_cell_route(impl, s, g);
            if (route.empty())
                continue; // disconnected components -> NoPath (SIM-001)
            std::vector<std::array<Vec2, 2>> portals;
            portals.reserve(route.size() - 1);
            for (std::size_t k = 0; k + 1 < route.size(); ++k)
                portals.push_back(detail::shared_portal(impl, route[k], route[k + 1]));
            const std::vector<Vec2> pulled = detail::string_pull(start, goal, portals);
            if (pulled.empty())
                continue;
            if (!detail::route_segments_contained(impl, pulled))
                continue; // SIM-002 gate: unverified routes are not returned
            const float total = detail::polyline_length(pulled);
            if (!found || total < best_length) // first candidate wins ties
            {
                found = true;
                best_length = total;
                best = Path{pulled};
            }
        }
    }
    if (!found)
        return detail::fail(ErrorCode::NoPath, "no contained route connects the endpoints");
    return best;
}

} // namespace vwmini
