// Navmesh and path track (N): triangle-mesh validation (MSH-004), topology
// (MSH-005), containment (MSH-006), and find_path direct/disconnected/bent behavior
// (SIM-001..SIM-004).
//
// This track uses fixed, hand-authored valid triangles; it never calls the
// triangulator.

#include "conformance_fixture.hpp"

#include <vwmini/nav_mesh.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using namespace vwmini;
using namespace vwmini_conformance;

namespace {

[[nodiscard]] Polygon tri(Vec2 a, Vec2 b, Vec2 c)
{
    return Polygon{{a, b, c}};
}

using ParameterInterval = std::pair<double, double>;

struct DoubleVec {
    double x;
    double y;
};

[[nodiscard]] DoubleVec to_double(Vec2 value)
{
    return {static_cast<double>(value.x), static_cast<double>(value.y)};
}

[[nodiscard]] DoubleVec operator-(DoubleVec left, DoubleVec right)
{
    return {left.x - right.x, left.y - right.y};
}

[[nodiscard]] double dot(DoubleVec left, DoubleVec right)
{
    return left.x * right.x + left.y * right.y;
}

[[nodiscard]] double cross(DoubleVec left, DoubleVec right)
{
    return left.x * right.y - left.y * right.x;
}

[[nodiscard]] bool is_finite(DoubleVec value)
{
    return std::isfinite(value.x) && std::isfinite(value.y);
}

[[nodiscard]] bool clip_linear_interval(double value_at_zero, double slope, double lower,
                                        double upper, ParameterInterval& interval)
{
    if (slope == 0.0) {
        return value_at_zero >= lower && value_at_zero <= upper;
    }

    const double first = (lower - value_at_zero) / slope;
    const double second = (upper - value_at_zero) / slope;
    interval.first = std::max(interval.first, std::min(first, second));
    interval.second = std::min(interval.second, std::max(first, second));
    return interval.first <= interval.second;
}

void append_circle_coverage(std::vector<ParameterInterval>& intervals, DoubleVec start,
                            DoubleVec delta, DoubleVec centre)
{
    const DoubleVec offset = start - centre;
    const double a = dot(delta, delta);
    const double b = 2.0 * dot(offset, delta);
    const double c = dot(offset, offset) - static_cast<double>(kEps) * kEps;
    const double discriminant = b * b - 4.0 * a * c;
    if (!std::isfinite(discriminant) || discriminant < 0.0) {
        return;
    }

    const double root = std::sqrt(discriminant);
    const double first = (-b - root) / (2.0 * a);
    const double second = (-b + root) / (2.0 * a);
    const ParameterInterval overlap{std::max(0.0, first), std::min(1.0, second)};
    if (overlap.first <= overlap.second) {
        intervals.push_back(overlap);
    }
}

/// Computes continuous parameter coverage for this suite's literal triangle geometry. A
/// point is covered when it is in a closed triangle or within epsilon of a closed triangle
/// edge, precisely matching MSH-006 for the non-degenerate CCW triangles below.
[[nodiscard]] bool segment_covered_by_triangles(const std::vector<Polygon>& triangles, Vec2 start,
                                                Vec2 end)
{
    const DoubleVec start_d = to_double(start);
    const DoubleVec delta = to_double(end) - start_d;
    const double delta_squared = dot(delta, delta);
    if (!is_finite(start_d) || !is_finite(delta) || delta_squared == 0.0) {
        return false;
    }

    std::vector<ParameterInterval> intervals;
    for (const Polygon& polygon : triangles) {
        const DoubleVec a = to_double(polygon.vertices[0]);
        const DoubleVec b = to_double(polygon.vertices[1]);
        const DoubleVec c = to_double(polygon.vertices[2]);

        ParameterInterval triangle_interval{0.0, 1.0};
        bool overlaps_triangle = true;
        for (const auto& [edge_start, edge_end] : {std::pair{a, b}, std::pair{b, c},
                                                    std::pair{c, a}}) {
            const DoubleVec edge = edge_end - edge_start;
            const DoubleVec offset = start_d - edge_start;
            if (!clip_linear_interval(cross(edge, offset), cross(edge, delta), 0.0,
                                      std::numeric_limits<double>::infinity(),
                                      triangle_interval)) {
                overlaps_triangle = false;
                break;
            }
        }
        if (overlaps_triangle) {
            intervals.push_back(triangle_interval);
        }

        for (const auto& [edge_start, edge_end] : {std::pair{a, b}, std::pair{b, c},
                                                    std::pair{c, a}}) {
            const DoubleVec edge = edge_end - edge_start;
            const double edge_squared = dot(edge, edge);
            ParameterInterval strip_interval{0.0, 1.0};
            const DoubleVec offset = start_d - edge_start;
            const double projection_at_zero = dot(offset, edge);
            const double projection_slope = dot(delta, edge);
            const double edge_length = std::sqrt(edge_squared);
            if (clip_linear_interval(projection_at_zero, projection_slope, 0.0, edge_squared,
                                     strip_interval) &&
                clip_linear_interval(cross(edge, offset), cross(edge, delta),
                                     -static_cast<double>(kEps) * edge_length,
                                     static_cast<double>(kEps) * edge_length,
                                     strip_interval)) {
                intervals.push_back(strip_interval);
            }
            append_circle_coverage(intervals, start_d, delta, edge_start);
            append_circle_coverage(intervals, start_d, delta, edge_end);
        }
    }

    std::sort(intervals.begin(), intervals.end());
    double covered_until = 0.0;
    for (const ParameterInterval interval : intervals) {
        if (interval.first > covered_until) {
            return false;
        }
        covered_until = std::max(covered_until, interval.second);
        if (covered_until >= 1.0) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] bool path_has_exactly_valid_irregular_segments(const Path& path,
                                                              const std::vector<Polygon>& triangles,
                                                              Vec2 start, Vec2 goal)
{
    if (path.points.empty() || path.points.front() != start || path.points.back() != goal) {
        return false;
    }
    for (const Vec2 point : path.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            return false;
        }
    }
    for (std::size_t i = 0; i + 1u < path.points.size(); ++i) {
        const DoubleVec delta = to_double(path.points[i + 1u]) - to_double(path.points[i]);
        const double distance_squared = dot(delta, delta);
        if (distance_squared < static_cast<double>(kEps) * kEps ||
            !segment_covered_by_triangles(triangles, path.points[i], path.points[i + 1u])) {
            return false;
        }
    }
    return true;
}

} // namespace

// ---- MSH-004: NavMesh::create validation and error codes ----

TEST(NavMesh_Create, EmptyListIsInvalidMesh)
{
    expect_error(NavMesh::create({}), ErrorCode::InvalidMesh);
}

TEST(NavMesh_Create, NonFiniteVertexIsInvalidArgument)
{
    expect_error(NavMesh::create({tri({0.0f, 0.0f}, {10.0f, 0.0f}, {std::nanf(""), 5.0f})}),
                 ErrorCode::InvalidArgument);
}

TEST(NavMesh_Create, NonTriangleIsInvalidMesh)
{
    expect_error(NavMesh::create({Polygon{{Vec2{0.0f, 0.0f}, Vec2{10.0f, 0.0f}}}}),
                 ErrorCode::InvalidMesh);
    expect_error(NavMesh::create({Polygon{{Vec2{0.0f, 0.0f}, Vec2{10.0f, 0.0f},
                                           Vec2{10.0f, 10.0f}, Vec2{0.0f, 10.0f}}}}),
                 ErrorCode::InvalidMesh);
}

TEST(NavMesh_Create, ClockwiseTriangleIsInvalidMesh)
{
    expect_error(NavMesh::create({tri({0.0f, 0.0f}, {0.0f, 10.0f}, {10.0f, 0.0f})}),
                 ErrorCode::InvalidMesh);
}

TEST(NavMesh_Create, DegenerateTriangleIsInvalidMesh)
{
    expect_error(NavMesh::create({tri({0.0f, 0.0f}, {5.0f, 0.0f}, {10.0f, 0.0f})}),
                 ErrorCode::InvalidMesh);
}

TEST(NavMesh_Create, OverlappingInteriorsIsInvalidMesh)
{
    // Two triangles whose interiors overlap (the second sits inside the first).
    expect_error(NavMesh::create({tri({0.0f, 0.0f}, {10.0f, 0.0f}, {0.0f, 10.0f}),
                                  tri({1.0f, 1.0f}, {3.0f, 1.0f}, {1.0f, 3.0f})}),
                 ErrorCode::InvalidMesh);
}

TEST(NavMesh_Create, TJunctionIsInvalidMesh)
{
    // A vertex of the small triangle lies in the interior of the large triangle's edge.
    expect_error(NavMesh::create({tri({0.0f, 0.0f}, {10.0f, 0.0f}, {0.0f, 10.0f}),
                                  tri({5.0f, 0.0f}, {10.0f, 0.0f}, {10.0f, 5.0f})}),
                 ErrorCode::InvalidMesh);
}

TEST(NavMesh_Create, NonManifoldEdgeIsInvalidMesh)
{
    // Three triangles all sharing the same complete edge.
    expect_error(NavMesh::create({tri({0.0f, 0.0f}, {10.0f, 0.0f}, {0.0f, 10.0f}),
                                  tri({0.0f, 0.0f}, {10.0f, 0.0f}, {10.0f, -10.0f}),
                                  tri({0.0f, 0.0f}, {10.0f, 0.0f}, {0.0f, -10.0f})}),
                 ErrorCode::InvalidMesh);
}

TEST(NavMesh_Create, ValidMeshReportsCellCount)
{
    const NavMesh mesh = make_square_mesh();
    EXPECT_EQ(mesh.cell_count(), 2u);
}

TEST(NavMesh_Create, DisjointComponentsAreValid)
{
    // MSH-005 / edge case: disjoint valid components are allowed.
    const NavMesh mesh = make_disconnected_mesh();
    EXPECT_EQ(mesh.cell_count(), 2u);
}

// ---- MSH-006: containment and boundary policy ----

TEST(NavMesh_Contains, NonFinitePointIsFalse)
{
    const NavMesh mesh = make_single_triangle_mesh();
    EXPECT_FALSE(mesh.contains(Vec2{std::nanf(""), 5.0f}));
    EXPECT_FALSE(mesh.contains(Vec2{5.0f, INFINITY}));
}

TEST(NavMesh_Contains, StrictlyInsideIsTrue)
{
    const NavMesh mesh = make_single_triangle_mesh();
    EXPECT_TRUE(mesh.contains(Vec2{2.0f, 2.0f}));
    EXPECT_TRUE(mesh.contains(Vec2{5.0f, 4.0f}));
}

TEST(NavMesh_Contains, BoundaryWithinEpsilonIsTrue)
{
    const NavMesh mesh = make_single_triangle_mesh();
    // Vertex and edge points.
    EXPECT_TRUE(mesh.contains(Vec2{0.0f, 0.0f}));
    EXPECT_TRUE(mesh.contains(Vec2{5.0f, 0.0f}));
    // Just outside the hypotenuse (x + y = 10) but within epsilon.
    EXPECT_TRUE(mesh.contains(Vec2{5.00005f, 5.00005f}));
}

TEST(NavMesh_Contains, SharedEdgePointIsContained)
{
    // The square's diagonal (0,0)-(10,10) is a shared complete edge; a point on it is
    // contained under MSH-006.
    const NavMesh mesh = make_square_mesh();
    EXPECT_TRUE(mesh.contains(Vec2{5.0f, 5.0f}));
    EXPECT_TRUE(mesh.contains(Vec2{2.5f, 2.5f}));
}

TEST(NavMesh_Contains, OutsideIsFalse)
{
    const NavMesh mesh = make_single_triangle_mesh();
    EXPECT_FALSE(mesh.contains(Vec2{5.01f, 5.01f}));
    EXPECT_FALSE(mesh.contains(Vec2{-0.001f, 5.0f}));
    EXPECT_FALSE(mesh.contains(Vec2{11.0f, 0.0f}));
}

TEST(NavMesh_Contains, SmallValidTriangleContainsItsInterior)
{
    // The signed double area is 4e-5, comfortably above epsilon^2 (1e-8), while
    // each strict-orientation cross is below the distance epsilon. Strict-inside
    // classification must use orientation, not misuse a distance tolerance as area.
    const auto result = NavMesh::create({tri({0.0f, 0.0f}, {0.01f, 0.0f}, {0.0f, 0.008f})});
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->contains(Vec2{0.002f, 0.002f}));
}

TEST(NavMesh_Contains, FiniteExtremeCoordinateTriangleContainsItsInterior)
{
    // Finite input remains in contract. Predicates must avoid overflowing float
    // intermediates for a large, but representable, accepted triangle.
    const float scale = std::numeric_limits<float>::max() / 4.0f;
    const auto result = NavMesh::create({tri({0.0f, 0.0f}, {scale, 0.0f}, {0.0f, scale})});
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->contains(Vec2{scale / 4.0f, scale / 4.0f}));
}

// ---- SIM-001: find_path endpoint validation and direct paths ----

TEST(FindPath, NonFiniteEndpointIsInvalidArgument)
{
    const NavMesh mesh = make_square_mesh();
    expect_error(find_path(mesh, Vec2{std::nanf(""), 5.0f}, Vec2{5.0f, 5.0f}),
                 ErrorCode::InvalidArgument);
    expect_error(find_path(mesh, Vec2{5.0f, 5.0f}, Vec2{5.0f, INFINITY}),
                 ErrorCode::InvalidArgument);
}

TEST(FindPath, OutsideEndpointIsOutsideMesh)
{
    const NavMesh mesh = make_square_mesh();
    expect_error(find_path(mesh, Vec2{-1.0f, 5.0f}, Vec2{5.0f, 5.0f}),
                 ErrorCode::OutsideMesh);
    expect_error(find_path(mesh, Vec2{5.0f, 5.0f}, Vec2{5.0f, 11.0f}),
                 ErrorCode::OutsideMesh);
}

TEST(FindPath, DisconnectedEndpointsIsNoPath)
{
    const NavMesh mesh = make_disconnected_mesh();
    expect_error(find_path(mesh, Vec2{5.0f, 2.0f}, Vec2{25.0f, 2.0f}),
                 ErrorCode::NoPath);
}

TEST(FindPath, DirectPathIsExactlyStartGoal)
{
    const NavMesh mesh = make_square_mesh();
    const auto result = find_path(mesh, Vec2{2.0f, 2.0f}, Vec2{8.0f, 8.0f});
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->points.size(), 2u);
    EXPECT_EQ(result->points[0], (Vec2{2.0f, 2.0f}));
    EXPECT_EQ(result->points[1], (Vec2{8.0f, 8.0f}));
}

TEST(FindPath, EqualEndpointsReturnSinglePoint)
{
    const NavMesh mesh = make_square_mesh();
    const auto result = find_path(mesh, Vec2{4.0f, 4.0f}, Vec2{4.0f, 4.0f});
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->points.size(), 1u);
    EXPECT_EQ(result->points[0], (Vec2{4.0f, 4.0f}));
}

TEST(FindPath, DistinctEndpointsWithinEpsilonRemainDirectPath)
{
    const NavMesh mesh = make_square_mesh();
    const Vec2 start{4.0f, 4.0f};
    const Vec2 goal{4.00005f, 4.0f};
    ASSERT_NE(start, goal);

    const auto result = find_path(mesh, start, goal);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->points.size(), 2u);
    EXPECT_EQ(result->points.front(), start);
    EXPECT_EQ(result->points.back(), goal);
}

TEST(FindPath, BoundaryToleranceDirectPathPreservesExactEndpoints)
{
    const NavMesh mesh = make_square_mesh();
    const Vec2 start{-0.00005f, 1.0f};
    const Vec2 goal{10.00005f, 9.0f};
    ASSERT_TRUE(mesh.contains(start));
    ASSERT_TRUE(mesh.contains(goal));

    const auto result = find_path(mesh, start, goal);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->points.size(), 2u);
    EXPECT_EQ(result->points.front(), start);
    EXPECT_EQ(result->points.back(), goal);
}

// ---- SIM-002 / SIM-003: connected and bent paths ----

TEST(FindPath, ConnectedPathPreservesEndpointsAndStaysContained)
{
    const NavMesh mesh = make_square_mesh();
    const auto result = find_path(mesh, Vec2{1.0f, 1.0f}, Vec2{9.0f, 9.0f});
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(path_is_valid(mesh, *result, Vec2{1.0f, 1.0f}, Vec2{9.0f, 9.0f}));
}

TEST(FindPath, VisibleMultiCellRouteIsExactlyDirect)
{
    std::vector<Polygon> triangles;
    // A convex 5x5 rectangle, each unit square divided consistently into two CCW cells.
    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 5; ++x) {
            const float left = static_cast<float>(x);
            const float bottom = static_cast<float>(y);
            triangles.push_back(tri({left, bottom}, {left + 1.0f, bottom},
                                    {left + 1.0f, bottom + 1.0f}));
            triangles.push_back(tri({left, bottom}, {left + 1.0f, bottom + 1.0f},
                                    {left, bottom + 1.0f}));
        }
    }
    const auto created = NavMesh::create(std::move(triangles));
    ASSERT_TRUE(created.has_value());
    const Vec2 start{4.82581f, 4.49509f};
    const Vec2 goal{2.73522f, 0.870023f};

    const auto result = find_path(*created, start, goal);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->points.size(), 2u);
    EXPECT_EQ(result->points.front(), start);
    EXPECT_EQ(result->points.back(), goal);
}

TEST(FindPath, BentPathShortensAroundReflexCorner)
{
    const NavMesh mesh = make_l_mesh();
    const Vec2 start{1.9f, 0.5f};
    const Vec2 goal{0.5f, 1.9f};

    // The straight segment cuts through the missing cell, so it is not covered.
    EXPECT_FALSE(segment_contained(mesh, start, goal));

    const auto result = find_path(mesh, start, goal);
    ASSERT_TRUE(result.has_value());
    // A bent route has at least one intermediate point.
    EXPECT_GT(result->points.size(), 2u);
    EXPECT_TRUE(path_is_valid(mesh, *result, start, goal));

    // The route must go around the reflex corner rather than straight through the gap.
    for (const Vec2 p : result->points) {
        EXPECT_TRUE(mesh.contains(p));
    }
}

TEST(FindPath, DeterministicAcrossRepeatedCalls)
{
    const NavMesh mesh = make_l_mesh();
    const Vec2 start{1.9f, 0.5f};
    const Vec2 goal{0.5f, 1.9f};
    const auto first = find_path(mesh, start, goal);
    const auto second = find_path(mesh, start, goal);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(first->points, second->points);
}

TEST(FindPath, IrregularConnectedMeshKeepsEverySegmentContained)
{
    // Five rows, listed bottom-to-top. All occupied cells are split into two CCW unit
    // triangles. The route below is connected but must turn around the missing cells.
    constexpr const char* rows[] = {"#####", "#.###", "#####", "###.#", "#####"};
    std::vector<Polygon> triangles;
    for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 5; ++x) {
            if (rows[y][x] != '#') {
                continue;
            }
            const float left = static_cast<float>(x);
            const float bottom = static_cast<float>(y);
            triangles.push_back(tri({left, bottom}, {left + 1.0f, bottom},
                                    {left + 1.0f, bottom + 1.0f}));
            triangles.push_back(tri({left, bottom}, {left + 1.0f, bottom + 1.0f},
                                    {left, bottom + 1.0f}));
        }
    }
    const auto created = NavMesh::create(triangles);
    ASSERT_TRUE(created.has_value());
    const Vec2 start{1.31f, 0.69f};
    const Vec2 goal{1.69f, 2.31f};
    ASSERT_TRUE(created->contains(start));
    ASSERT_TRUE(created->contains(goal));
    EXPECT_FALSE(segment_covered_by_triangles(triangles, start, goal));

    const auto result = find_path(*created, start, goal);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(path_has_exactly_valid_irregular_segments(*result, triangles, start, goal));
}
