#include "geometry_internal.hpp"
#include "mesh_internal.hpp"

#include <vwmini/nav_mesh.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

namespace vwmini {
namespace {

using geom::is_finite;
using geom::kEpsilon;
using geom::kEpsilonSquared;
using geom::orient;

[[nodiscard]] Error fail(ErrorCode code, std::string message)
{
    return Error{code, std::move(message)};
}

[[nodiscard]] bool near(Vec2 a, Vec2 b) noexcept
{
    const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
    const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
    return dx * dx + dy * dy <= kEpsilonSquared;
}

[[nodiscard]] double point_segment_distance(Vec2 point, Vec2 a, Vec2 b) noexcept
{
    const double ab_x = static_cast<double>(b.x) - static_cast<double>(a.x);
    const double ab_y = static_cast<double>(b.y) - static_cast<double>(a.y);
    const double ap_x = static_cast<double>(point.x) - static_cast<double>(a.x);
    const double ap_y = static_cast<double>(point.y) - static_cast<double>(a.y);
    const double length_squared = ab_x * ab_x + ab_y * ab_y;
    const double t = length_squared == 0.0
                         ? 0.0
                         : std::clamp((ap_x * ab_x + ap_y * ab_y) / length_squared, 0.0, 1.0);
    const double dx = ap_x - t * ab_x;
    const double dy = ap_y - t * ab_y;
    return std::sqrt(dx * dx + dy * dy);
}

[[nodiscard]] bool contains_triangle(const Polygon& triangle, Vec2 point) noexcept
{
    const Vec2 a = triangle.vertices[0];
    const Vec2 b = triangle.vertices[1];
    const Vec2 c = triangle.vertices[2];
    if (orient(a, b, point) > 0.0 && orient(b, c, point) > 0.0 &&
        orient(c, a, point) > 0.0) {
        return true;
    }
    return point_segment_distance(point, a, b) <= kEpsilon ||
           point_segment_distance(point, b, c) <= kEpsilon ||
           point_segment_distance(point, c, a) <= kEpsilon;
}

[[nodiscard]] std::size_t containing_triangle(const auto& mesh, Vec2 point) noexcept
{
    for (std::size_t index = 0; index < mesh.triangles.size(); ++index) {
        if (contains_triangle(mesh.triangles[index], point)) {
            return index;
        }
    }
    return kNoNeighbor;
}

struct Interval {
    double first{};
    double last{};
};

// Computes the part of a line segment inside one closed CCW triangle. The three
// half-plane constraints are clipped parametrically, avoiding sampling artefacts.
[[nodiscard]] bool triangle_interval(const Polygon& triangle, Vec2 start, Vec2 end,
                                     Interval& result) noexcept
{
    const Vec2 delta = end - start;
    double low = 0.0;
    double high = 1.0;

    for (std::size_t edge = 0; edge < 3; ++edge) {
        const Vec2 a = triangle.vertices[edge];
        const Vec2 b = triangle.vertices[(edge + 1u) % 3u];
        const double edge_x = static_cast<double>(b.x) - static_cast<double>(a.x);
        const double edge_y = static_cast<double>(b.y) - static_cast<double>(a.y);
        // orient has area units. Convert MSH-006's distance tolerance to the
        // corresponding signed-area tolerance for this edge's half-plane.
        const double boundary_tolerance = kEpsilon * std::sqrt(edge_x * edge_x + edge_y * edge_y);
        const double constant = orient(a, b, start) + boundary_tolerance;
        const double coefficient = edge_x * delta.y - edge_y * delta.x;
        if (coefficient == 0.0) {
            if (constant < 0.0) {
                return false;
            }
            continue;
        }
        const double crossing = -constant / coefficient;
        if (coefficient > 0.0) {
            low = std::max(low, crossing);
        } else {
            high = std::min(high, crossing);
        }
        if (low > high) {
            return false;
        }
    }

    result = {std::max(0.0, low), std::min(1.0, high)};
    return result.first <= result.last;
}

[[nodiscard]] bool segment_is_covered(const auto& mesh, Vec2 start, Vec2 end) noexcept
{
    std::vector<Interval> intervals;
    intervals.reserve(mesh.triangles.size());
    for (const Polygon& triangle : mesh.triangles) {
        Interval interval;
        if (triangle_interval(triangle, start, end, interval)) {
            intervals.push_back(interval);
        }
    }
    if (intervals.empty()) {
        return false;
    }
    std::ranges::sort(intervals, {}, &Interval::first);
    if (intervals.front().first > 0.0) {
        return false;
    }
    double covered_until = intervals.front().last;
    for (std::size_t index = 1; index < intervals.size() && covered_until < 1.0; ++index) {
        if (intervals[index].first > covered_until + 1e-10) {
            return false;
        }
        covered_until = std::max(covered_until, intervals[index].last);
    }
    return covered_until >= 1.0;
}

[[nodiscard]] std::vector<std::size_t> corridor(const auto& mesh, std::size_t start,
                                                 std::size_t goal)
{
    std::vector<std::size_t> parent(mesh.triangles.size(), kNoNeighbor);
    std::queue<std::size_t> pending;
    parent[start] = start;
    pending.push(start);

    while (!pending.empty()) {
        const std::size_t current = pending.front();
        pending.pop();
        if (current == goal) {
            break;
        }
        std::array<std::size_t, 3> neighbors = mesh.neighbors[current];
        std::ranges::sort(neighbors);
        for (const std::size_t next : neighbors) {
            if (next == kNoNeighbor || parent[next] != kNoNeighbor) {
                continue;
            }
            parent[next] = current;
            pending.push(next);
        }
    }
    if (parent[goal] == kNoNeighbor) {
        return {};
    }

    std::vector<std::size_t> result;
    for (std::size_t current = goal;; current = parent[current]) {
        result.push_back(current);
        if (current == start) {
            break;
        }
    }
    std::ranges::reverse(result);
    return result;
}

void append_unique_vertex(std::vector<Vec2>& vertices, Vec2 point)
{
    if (std::ranges::none_of(vertices, [point](Vec2 existing) { return near(existing, point); })) {
        vertices.push_back(point);
    }
}

// The benchmark has no large-mesh performance target. A visibility graph over the
// mesh's existing vertices makes the reference route geometrically useful: shortest
// paths bend at obstacle corners rather than at arbitrary portal midpoints.
[[nodiscard]] std::vector<Vec2> shortest_visibility_path(const auto& mesh, Vec2 start, Vec2 goal)
{
    std::vector<Vec2> vertices{start, goal};
    for (const Polygon& triangle : mesh.triangles) {
        for (const Vec2 vertex : triangle.vertices) {
            append_unique_vertex(vertices, vertex);
        }
    }

    const std::size_t count = vertices.size();
    std::vector<double> distance(count, std::numeric_limits<double>::infinity());
    std::vector<std::size_t> parent(count, kNoNeighbor);
    std::vector<bool> settled(count, false);
    distance[0] = 0.0;

    for (std::size_t iteration = 0; iteration < count; ++iteration) {
        std::size_t current = kNoNeighbor;
        for (std::size_t candidate = 0; candidate < count; ++candidate) {
            if (!settled[candidate] && (current == kNoNeighbor ||
                distance[candidate] < distance[current] - 1e-12 ||
                (std::abs(distance[candidate] - distance[current]) <= 1e-12 && candidate < current))) {
                current = candidate;
            }
        }
        if (current == kNoNeighbor || !std::isfinite(distance[current])) {
            break;
        }
        if (current == 1u) {
            break;
        }
        settled[current] = true;

        for (std::size_t candidate = 0; candidate < count; ++candidate) {
            if (candidate == current || settled[candidate] ||
                !segment_is_covered(mesh, vertices[current], vertices[candidate])) {
                continue;
            }
            const double candidate_distance = distance[current] +
                std::hypot(static_cast<double>(vertices[current].x) - vertices[candidate].x,
                           static_cast<double>(vertices[current].y) - vertices[candidate].y);
            if (candidate_distance < distance[candidate] - 1e-12 ||
                (std::abs(candidate_distance - distance[candidate]) <= 1e-12 &&
                 (parent[candidate] == kNoNeighbor || current < parent[candidate]))) {
                distance[candidate] = candidate_distance;
                parent[candidate] = current;
            }
        }
    }

    if (!std::isfinite(distance[1u])) {
        return {};
    }
    std::vector<Vec2> result;
    for (std::size_t current = 1u;; current = parent[current]) {
        result.push_back(vertices[current]);
        if (current == 0u) {
            break;
        }
    }
    std::ranges::reverse(result);
    return result;
}

} // namespace

[[nodiscard]] Result<Path> find_path(const NavMesh& mesh, Vec2 start, Vec2 goal)
{
    if (!is_finite(start) || !is_finite(goal)) {
        return std::unexpected(fail(ErrorCode::InvalidArgument, "non-finite path endpoint"));
    }
    const auto& state = *mesh.m_impl;
    const std::size_t start_cell = containing_triangle(state, start);
    const std::size_t goal_cell = containing_triangle(state, goal);
    if (start_cell == kNoNeighbor || goal_cell == kNoNeighbor) {
        return std::unexpected(fail(ErrorCode::OutsideMesh, "path endpoint outside mesh"));
    }
    if (start == goal) {
        return Path{{start}};
    }
    if (segment_is_covered(state, start, goal)) {
        return Path{{start, goal}};
    }

    const std::vector<std::size_t> cells = corridor(state, start_cell, goal_cell);
    if (cells.empty()) {
        return std::unexpected(fail(ErrorCode::NoPath, "disconnected path endpoints"));
    }

    const std::vector<Vec2> points = shortest_visibility_path(state, start, goal);
    if (points.empty()) {
        return std::unexpected(fail(ErrorCode::NoPath, "no contained visibility route"));
    }
    return Path{points};
}

} // namespace vwmini
