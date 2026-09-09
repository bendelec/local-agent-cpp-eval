#include <vwmini/nav_mesh.hpp>

#include "vwmini/internal/errors.hpp"
#include "vwmini/internal/mesh_data.hpp"
#include "vwmini/internal/predicates.hpp"
#include "vwmini/internal/vec2d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <vector>

// Deterministic point-to-point routing (SIM-001..004).
//
// Strategy:
//  1. Validate endpoints and locate their cells (first match in caller order).
//  2. If the straight segment start->goal is contained in the mesh (proved exactly
//     by covering the parameter interval [0,1] with per-triangle convex intervals),
//     return [start, goal] directly (SIM-001).
//  3. Otherwise search a corridor of adjacent cells with deterministic Dijkstra
//     (weight: portal-midpoint polyline length; ties keep the lower cell index),
//     then tighten it to the shortest contained polyline with the funnel algorithm
//     (SIM-002/003). All internal geometry runs in double on float-derived values,
//     so identical input yields bitwise-identical output on one platform (SIM-004).

namespace vwmini {
namespace {

using internal::Cell;
using internal::D2;
using internal::Index3;
using internal::cell_contains;
using internal::cross2;
using internal::dist;
using internal::distance;
using internal::dot2;
using internal::is_finite;
using internal::kEpsD;
using internal::make_error;
using internal::next_corner;
using internal::to_d;
using internal::triarea2;

// -- Closed sub-intervals of [0,1] --------------------------------------------

/// A closed interval [lo, hi] of the segment parameter t; lo > hi means empty.
struct Interval {
    double lo{};
    double hi{};

    [[nodiscard]] bool valid() const noexcept
    {
        return lo <= hi;
    }
};

constexpr Interval kEmptyInterval{1.0, 0.0};

[[nodiscard]] Interval intersect(Interval a, Interval b) noexcept
{
    if (!a.valid() || !b.valid()) {
        return kEmptyInterval;
    }
    const Interval r{std::max(a.lo, b.lo), std::min(a.hi, b.hi)};
    return r.valid() ? r : kEmptyInterval;
}

/// Smallest interval containing both inputs. Used only where the union is convex,
/// so the hull equals the union (distance-to-convex-set along a line is convex).
[[nodiscard]] Interval hull(Interval a, Interval b) noexcept
{
    if (!a.valid()) {
        return b;
    }
    if (!b.valid()) {
        return a;
    }
    return {std::min(a.lo, b.lo), std::max(a.hi, b.hi)};
}

/// {t in [0,1] : f0 + f1*t >= 0} for an affine function of t.
[[nodiscard]] Interval affine_ge(double f0, double f1) noexcept
{
    if (f1 == 0.0) {
        return f0 >= 0.0 ? Interval{0.0, 1.0} : kEmptyInterval;
    }
    const double root = -f0 / f1;
    const Interval r =
        f1 > 0.0 ? Interval{std::max(0.0, root), 1.0} : Interval{0.0, std::min(1.0, root)};
    return r.valid() ? r : kEmptyInterval;
}

// -- Direct-segment containment (SIM-001) --------------------------------------
//
// `triangle_contains_eps(p, T)` is equivalent to dist(p, closed triangle T) <= eps,
// whose solution set is the convex epsilon-dilation of T. Along the segment
// p(t) = start + t*(goal - start) every "within epsilon of T" condition is therefore
// a single closed t-interval, computed exactly below:
//   * closed triangle interior: three affine half-plane conditions;
//   * epsilon band around each edge: a capsule (two endpoint disks plus the
//     perpendicular band over the projection interval).
// The union over all triangles covers [0,1] if and only if the whole segment is
// contained under MSH-006.

/// {t in [0,1] : dist(start + t*d, closed segment [a,b]) <= eps}; a != b.
[[nodiscard]] Interval capsule_interval(D2 s, D2 d, D2 a, D2 b) noexcept
{
    /// {t in [0,1] : |s + t*d - c| <= eps}: one quadratic inequality.
    const auto disk = [&](D2 c) -> Interval {
        const D2 o = s - c;
        const double qa = dot2(d, d); // > 0: start and goal differ in at least one bit.
        const double qb = 2.0 * dot2(d, o);
        const double qc = dot2(o, o) - kEpsD * kEpsD;
        const double disc = qb * qb - 4.0 * qa * qc;
        if (disc < 0.0) {
            return kEmptyInterval;
        }
        const double root = std::sqrt(disc);
        const Interval r{std::max(0.0, (-qb - root) / (2.0 * qa)),
                         std::min(1.0, (-qb + root) / (2.0 * qa))};
        return r.valid() ? r : kEmptyInterval;
    };

    const D2 e = b - a;
    const double len2 = dot2(e, e); // > eps^2 > 0: welded mesh corners are distinct.
    const double len = std::sqrt(len2);
    const D2 sa = s - a;
    // Projection coordinate u(t) along the edge and signed perpendicular offset w(t);
    // both affine in t. The band covers the points projecting inside the segment.
    const double u0 = dot2(sa, e) / len2;
    const double u1 = dot2(d, e) / len2;
    const double w0 = cross2(sa, e) / len;
    const double w1 = cross2(d, e) / len;
    Interval band = intersect(affine_ge(u0, u1), affine_ge(1.0 - u0, -u1));
    band = intersect(band, affine_ge(w0 + kEpsD, w1));  // w >= -eps
    band = intersect(band, affine_ge(kEpsD - w0, -w1)); // w <= +eps

    return hull(hull(disk(a), disk(b)), band);
}

/// {t in [0,1] : triangle_contains_eps(start + t*d, c)}.
[[nodiscard]] Interval triangle_interval(D2 s, D2 d, const Cell& c) noexcept
{
    const D2 p0 = to_d(c[0]);
    const D2 p1 = to_d(c[1]);
    const D2 p2 = to_d(c[2]);
    // Closed interior: left of (or on) every directed edge of the CCW cell.
    const auto halfplane = [&](D2 a, D2 b) -> Interval {
        const D2 e = b - a;
        // orient(a, b, s + t*d) = cross2(e, s - a) + t * cross2(e, d).
        return affine_ge(cross2(e, s - a), cross2(e, d));
    };
    Interval r = intersect(halfplane(p0, p1), halfplane(p1, p2));
    r = intersect(r, halfplane(p2, p0));
    r = hull(r, capsule_interval(s, d, p0, p1));
    r = hull(r, capsule_interval(s, d, p1, p2));
    r = hull(r, capsule_interval(s, d, p2, p0));
    return r;
}

/// True when every point of the closed segment [start, goal] lies in some cell
/// under the MSH-006 boundary policy. Assumes both endpoints are finite and
/// start != goal (exact comparison).
[[nodiscard]] bool segment_covered(std::span<const Cell> cells, Vec2 start, Vec2 goal)
{
    const D2 s = to_d(start);
    const D2 d = to_d(goal) - s;
    std::vector<Interval> intervals;
    intervals.reserve(cells.size());
    for (const Cell& c : cells) {
        if (const Interval r = triangle_interval(s, d, c); r.valid()) {
            intervals.push_back(r);
        }
    }
    std::ranges::sort(
        intervals, [](Interval a, Interval b) { return a.lo == b.lo ? a.hi < b.hi : a.lo < b.lo; });
    // Merge walk: invariant "[0, covered_hi] is covered" (starts at 0 because the
    // caller has verified that start itself is contained).
    double covered_hi = 0.0;
    for (const Interval iv : intervals) {
        if (iv.lo > covered_hi) {
            return false; // a gap opens before this interval
        }
        covered_hi = std::max(covered_hi, iv.hi);
        if (covered_hi >= 1.0) {
            return true;
        }
    }
    return false;
}

// -- Cell location and corridor search (SIM-001, SIM-004) -----------------------

/// The first cell (caller order) containing `point`, or nullopt. Matches
/// `NavMesh::contains` semantics exactly.
[[nodiscard]] std::optional<std::size_t> locate_cell(std::span<const Cell> cells,
                                                     Vec2 point) noexcept
{
    const auto it =
        std::ranges::find_if(cells, [point](const Cell& c) { return cell_contains(point, c); });
    if (it == cells.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(it - cells.begin());
}

/// One corridor transition: the shared edge seen from the walking direction,
/// `left` strictly left of travel. Both points are exact mesh corners.
struct Portal {
    Vec2 left;
    Vec2 right;
};

/// Walkable cell chain from `start_cell` to `goal_cell` as the portals crossed,
/// or nullopt when the cells lie in disconnected components (SIM-001 NoPath).
///
/// Deterministic Dijkstra: cost is the polyline length through entered portal
/// midpoints; the unsettled cell with the smallest (cost, index) pair settles
/// next; equal-cost relaxations keep the first-found predecessor; neighbors are
/// scanned in fixed edge order 0, 1, 2.
[[nodiscard]] std::optional<std::vector<Portal>> search_corridor(std::span<const Cell> cells,
                                                                 std::span<const Index3> adjacency,
                                                                 std::size_t start_cell, Vec2 start,
                                                                 std::size_t goal_cell)
{
    const std::size_t n = cells.size();
    constexpr std::size_t kNoCell = std::numeric_limits<std::size_t>::max();
    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> cost(n, kInf);
    std::vector<D2> entry(n); // midpoint of the portal this cell was entered through
    std::vector<std::size_t> prev_cell(n, kNoCell);
    std::vector<std::size_t> prev_edge(n); // meaningful only when prev_cell != kNoCell
    std::vector<char> settled(n, 0);

    cost[start_cell] = 0.0;
    entry[start_cell] = to_d(start);

    while (true) {
        std::size_t i = kNoCell;
        for (std::size_t c = 0; c < n; ++c) {
            if (settled[c] != 0) {
                continue;
            }
            if (i == kNoCell || cost[c] < cost[i]) {
                i = c; // strict <: ties keep the lower index
            }
        }
        if (i == kNoCell || cost[i] == kInf) {
            return std::nullopt; // remaining cells are unreachable from start
        }
        if (i == goal_cell) {
            break; // cost is final by the Dijkstra invariant
        }
        settled[i] = 1;
        const Cell& ci = cells[i];
        for (std::size_t k = 0; k < 3; ++k) {
            const std::int32_t neighbor = adjacency[i][k];
            if (neighbor < 0) {
                continue; // boundary edge
            }
            const std::size_t j = static_cast<std::size_t>(neighbor);
            if (settled[j] != 0) {
                continue;
            }
            const D2 mid = (to_d(ci[k]) + to_d(ci[next_corner(k)])) * 0.5;
            const double candidate = cost[i] + dist(entry[i], mid);
            if (candidate < cost[j]) {
                cost[j] = candidate;
                entry[j] = mid;
                prev_cell[j] = i;
                prev_edge[j] = k;
            }
        }
    }

    // Unwind the predecessor chain. Walking cell p -> c through edge e of the CCW
    // cell p (from p[e] to p[e+1]) leaves p[e] on the right and p[e+1] on the left.
    std::vector<Portal> portals;
    for (std::size_t c = goal_cell; c != start_cell;) {
        const std::size_t p = prev_cell[c];
        if (p == kNoCell) {
            return std::nullopt; // unreachable: settled cells always have a predecessor
        }
        const std::size_t e = prev_edge[c];
        const Cell& cp = cells[p];
        portals.push_back(Portal{cp[next_corner(e)], cp[e]});
        c = p;
    }
    std::ranges::reverse(portals);
    return portals;
}

// -- Funnel string pulling (SIM-002, SIM-003) -----------------------------------

/// Interior bend points of the shortest polyline from `start` to `goal` through the
/// corridor portals (simple funnel algorithm, exact double orientation tests).
/// Portal vertices are exact mesh corners, and distinct welded corners differ by
/// more than epsilon, so the funnel's vertex-equality test is exact comparison of
/// the double images of identical float values.
[[nodiscard]] std::vector<Vec2> funnel_corners(Vec2 start, Vec2 goal,
                                               std::span<const Portal> portals)
{
    const std::size_t m = portals.size() + 2;
    std::vector<D2> left(m);
    std::vector<D2> right(m);
    std::vector<Vec2> left_corner(m);
    std::vector<Vec2> right_corner(m);
    left[0] = right[0] = to_d(start);
    left_corner[0] = right_corner[0] = start;
    left[m - 1] = right[m - 1] = to_d(goal);
    left_corner[m - 1] = right_corner[m - 1] = goal;
    for (std::size_t k = 0; k < portals.size(); ++k) {
        left[k + 1] = to_d(portals[k].left);
        right[k + 1] = to_d(portals[k].right);
        left_corner[k + 1] = portals[k].left;
        right_corner[k + 1] = portals[k].right;
    }

    std::vector<Vec2> corners;
    D2 apex = to_d(start);
    D2 lv = apex;
    D2 rv = apex;
    std::size_t left_i = 0;
    std::size_t right_i = 0;
    std::size_t i = 1;
    /// Emit the vertex blocking the line of sight as a corner, pivot the funnel
    /// onto it, and rescan the portals after it (++i in the loop applies).
    const auto pivot = [&](std::size_t side_i, Vec2 corner, D2 side) {
        corners.push_back(corner);
        apex = side;
        lv = apex;
        rv = apex;
        left_i = side_i;
        right_i = side_i;
        i = side_i;
    };
    for (; i < m; ++i) {
        if (triarea2(apex, rv, right[i]) >= 0.0) { // right vertex pinches or is collinear
            if (apex == rv || triarea2(apex, lv, right[i]) < 0.0) {
                rv = right[i];
                right_i = i;
            } else { // line of sight blocked on the left: pivot around the left vertex
                pivot(left_i, left_corner[left_i], lv);
                continue;
            }
        }
        if (triarea2(apex, lv, left[i]) <= 0.0) { // left vertex pinches or is collinear
            if (apex == lv || triarea2(apex, rv, left[i]) > 0.0) {
                lv = left[i];
                left_i = i;
            } else { // line of sight blocked on the right: pivot around the right vertex
                pivot(right_i, right_corner[right_i], rv);
                continue;
            }
        }
    }
    return corners;
}

/// Builds the final polyline: exact start, the interior corners that are at least
/// epsilon away from the previously kept point and from the goal (SIM-002 omission
/// rule), and the exact goal.
[[nodiscard]] Path assemble_path(Vec2 start, Vec2 goal, std::span<const Vec2> corners)
{
    std::vector<Vec2> points;
    points.reserve(corners.size() + 2);
    points.push_back(start);
    for (const Vec2 c : corners) {
        if (distance(points.back(), c) < kEpsD || distance(c, goal) < kEpsD) {
            continue;
        }
        points.push_back(c);
    }
    points.push_back(goal);
    return Path{std::move(points)};
}

} // namespace

Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal)
{
    if (!is_finite(start) || !is_finite(goal)) {
        return make_error(ErrorCode::InvalidArgument, "path endpoint is not finite");
    }
    // A moved-from NavMesh has a null impl and no cells; report the start as
    // outside (identical to an empty mesh) rather than dereferencing null below
    // at mesh.m_impl->adjacency. find_path is NavMesh's only friend, so it must
    // guard the access itself.
    if (!mesh.m_impl) {
        return make_error(ErrorCode::OutsideMesh, "path start is outside the mesh");
    }
    const std::span<const Cell> cells = mesh.m_impl->cells;
    const std::optional<std::size_t> start_cell = locate_cell(cells, start);
    if (!start_cell) {
        return make_error(ErrorCode::OutsideMesh, "path start is outside the mesh");
    }
    const std::optional<std::size_t> goal_cell = locate_cell(cells, goal);
    if (!goal_cell) {
        return make_error(ErrorCode::OutsideMesh, "path goal is outside the mesh");
    }
    if (start == goal) {
        return Path{std::vector<Vec2>{start}};
    }
    if (segment_covered(cells, start, goal)) {
        return Path{std::vector<Vec2>{start, goal}};
    }
    auto corridor = search_corridor(cells, mesh.m_impl->adjacency, *start_cell, start, *goal_cell);
    if (!corridor) {
        return make_error(ErrorCode::NoPath, "start and goal lie in disconnected mesh components");
    }
    return assemble_path(start, goal, funnel_corners(start, goal, *corridor));
}

} // namespace vwmini
