// Navmesh and path track (N): triangle-mesh validation (MSH-004), topology
// (MSH-005), containment (MSH-006), and find_path direct/disconnected/bent behavior
// (SIM-001..SIM-004).
//
// This track uses fixed, hand-authored valid triangles; it never calls the
// triangulator.

#include "conformance_fixture.hpp"

#include <vwmini/nav_mesh.hpp>

#include <cmath>
#include <vector>

using namespace vwmini;
using namespace vwmini_conformance;

namespace {

[[nodiscard]] Polygon tri(Vec2 a, Vec2 b, Vec2 c)
{
    return Polygon{{a, b, c}};
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
