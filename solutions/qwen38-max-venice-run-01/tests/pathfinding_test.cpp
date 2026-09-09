#include <vwmini/nav_mesh.hpp>

#include "test_util.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

namespace {

using namespace vwmini::test;
using vwmini::ErrorCode;
using vwmini::NavMesh;
using vwmini::Path;
using vwmini::Polygon;
using vwmini::Vec2;

// -- Fixture helpers ----------------------------------------------------------
// make_mesh / square_triangles / outline_triangles / make_l_mesh live in test_util.hpp.

void append(std::vector<Polygon>& into, std::vector<Polygon> more)
{
    std::ranges::move(more, std::back_inserter(into));
}

// A row of `count` touching 2x2 squares: [0, 2*count] x [0, 2].
NavMesh make_rectangle_mesh(int count)
{
    std::vector<Polygon> triangles;
    for (int i = 0; i < count; ++i) {
        append(triangles, square_triangles(V(2.0f * float(i), 0.0f), 2.0f));
    }
    return make_mesh(std::move(triangles));
}

// Mirrored L: bottom arm [0,2]x[0,1] plus vertical arm [1,2]x[0,3]; reflex (1,1).
NavMesh make_mirrored_l_mesh()
{
    return make_mesh(outline_triangles({V(0, 0), V(2, 0), V(2, 3), V(1, 3), V(1, 1), V(0, 1)}));
}

void expect_points_eq(const std::vector<Vec2>& actual, const std::vector<Vec2>& expected)
{
    ASSERT_EQ(actual.size(), expected.size()) << "point count differs";
    for (std::size_t i = 0; i < actual.size(); ++i) {
        EXPECT_EQ(actual[i], expected[i]) << "point " << i << " differs";
    }
}

// SIM-002: every sampled point of every segment is contained by the mesh.
void expect_path_contained(const NavMesh& mesh, const Path& path)
{
    constexpr int kSamples = 256;
    for (std::size_t s = 0; s + 1 < path.points.size(); ++s) {
        const Vec2 a = path.points[s];
        const Vec2 b = path.points[s + 1];
        for (int i = 0; i <= kSamples; ++i) {
            const float t = float(i) / float(kSamples);
            const Vec2 p = a + (b - a) * t;
            EXPECT_TRUE(mesh.contains(p)) << "segment " << s << " at t=" << t << " point (" << p.x
                                          << ", " << p.y << ") is outside the mesh";
        }
    }
}

// -- SIM-001: validation -------------------------------------------------------

TEST(FindPath, NonFiniteStartIsInvalidArgument)
{
    const NavMesh mesh = make_rectangle_mesh(1);
    auto path = find_path(mesh, V(NAN, 0.0f), V(1, 1));
    ASSERT_FALSE(path.has_value());
    EXPECT_EQ(code_of(path), ErrorCode::InvalidArgument);
}

TEST(FindPath, NonFiniteGoalIsInvalidArgument)
{
    const NavMesh mesh = make_rectangle_mesh(1);
    auto path = find_path(mesh, V(1, 1), V(0.0f, std::numeric_limits<float>::infinity()));
    ASSERT_FALSE(path.has_value());
    EXPECT_EQ(code_of(path), ErrorCode::InvalidArgument);
}

TEST(FindPath, NonFiniteEndpointPrecedesOutsideMesh)
{
    const NavMesh mesh = make_rectangle_mesh(1);
    auto path = find_path(mesh, V(NAN, NAN), V(1000, 1000));
    ASSERT_FALSE(path.has_value());
    EXPECT_EQ(code_of(path), ErrorCode::InvalidArgument);
}

TEST(FindPath, FiniteStartOutsideIsOutsideMesh)
{
    const NavMesh mesh = make_rectangle_mesh(1);
    auto path = find_path(mesh, V(-1, -1), V(1, 1));
    ASSERT_FALSE(path.has_value());
    EXPECT_EQ(code_of(path), ErrorCode::OutsideMesh);
}

TEST(FindPath, FiniteGoalOutsideIsOutsideMesh)
{
    const NavMesh mesh = make_rectangle_mesh(1);
    auto path = find_path(mesh, V(1, 1), V(5, 5));
    ASSERT_FALSE(path.has_value());
    EXPECT_EQ(code_of(path), ErrorCode::OutsideMesh);
}

TEST(FindPath, EqualEndpointsOutsideMeshIsOutsideMesh)
{
    const NavMesh mesh = make_rectangle_mesh(1);
    auto path = find_path(mesh, V(9, 9), V(9, 9));
    ASSERT_FALSE(path.has_value());
    EXPECT_EQ(code_of(path), ErrorCode::OutsideMesh);
}

TEST(FindPath, ExactlyEqualEndpointsReturnSinglePoint)
{
    const NavMesh mesh = make_rectangle_mesh(1);
    const Vec2 p = V(0.5f, 1.5f);
    auto path = find_path(mesh, p, p);
    ASSERT_TRUE(path.has_value());
    expect_points_eq(path->points, {p});
}

// -- SIM-001: direct paths ------------------------------------------------------

TEST(FindPath, ContainedSegmentInOneTriangleReturnsTwoExactPoints)
{
    const NavMesh mesh = make_mesh({Tri(V(0, 0), V(10, 0), V(0, 10))});
    const Vec2 start = V(1, 1);
    const Vec2 goal = V(2, 3);
    auto path = find_path(mesh, start, goal);
    ASSERT_TRUE(path.has_value());
    expect_points_eq(path->points, {start, goal});
}

TEST(FindPath, ContainedSegmentAcrossTwoCellsReturnsTwoExactPoints)
{
    const NavMesh mesh = make_rectangle_mesh(2); // [0,4] x [0,2]
    const Vec2 start = V(0.5f, 0.5f);
    const Vec2 goal = V(3.5f, 1.5f);
    auto path = find_path(mesh, start, goal);
    ASSERT_TRUE(path.has_value());
    expect_points_eq(path->points, {start, goal});
    expect_path_contained(mesh, *path);
}

TEST(FindPath, ContainedSegmentAcrossThreeCellsReturnsTwoExactPoints)
{
    const NavMesh mesh = make_rectangle_mesh(3); // [0,6] x [0,2]
    const Vec2 start = V(0.5f, 1.0f);
    const Vec2 goal = V(5.5f, 1.0f); // crosses both portals at interior points
    auto path = find_path(mesh, start, goal);
    ASSERT_TRUE(path.has_value());
    expect_points_eq(path->points, {start, goal});
}

TEST(FindPath, StartAtReflexCornerKeepsDirectPath)
{
    const NavMesh mesh = make_l_mesh();
    const Vec2 start = V(1, 1); // on the mesh boundary: contained per MSH-006
    const Vec2 goal = V(0.5f, 2.5f);
    auto path = find_path(mesh, start, goal);
    ASSERT_TRUE(path.has_value());
    expect_points_eq(path->points, {start, goal});
    expect_path_contained(mesh, *path);
}

// -- SIM-001: disconnected components --------------------------------------------

TEST(FindPath, DisjointComponentsReturnNoPath)
{
    std::vector<Polygon> triangles = square_triangles(V(0, 0), 2.0f);
    append(triangles, square_triangles(V(5, 5), 2.0f));
    const NavMesh mesh = make_mesh(std::move(triangles));
    auto path = find_path(mesh, V(0.5f, 0.5f), V(5.5f, 5.5f));
    ASSERT_FALSE(path.has_value());
    EXPECT_EQ(code_of(path), ErrorCode::NoPath);
}

TEST(FindPath, VertexOnlyTouchIsNotWalkable)
{
    std::vector<Polygon> triangles = square_triangles(V(0, 0), 2.0f);
    append(triangles, square_triangles(V(2, 2), 2.0f)); // touches at the corner (2,2)
    const NavMesh mesh = make_mesh(std::move(triangles));
    auto path = find_path(mesh, V(0.5f, 0.5f), V(3.5f, 2.5f));
    ASSERT_FALSE(path.has_value());
    EXPECT_EQ(code_of(path), ErrorCode::NoPath);
}

// -- SIM-002 / SIM-003: corridor paths -------------------------------------------

TEST(FindPath, LShapeBendsExactlyAtReflexCorner)
{
    const NavMesh mesh = make_l_mesh();
    const Vec2 start = V(1.5f, 0.5f);
    const Vec2 goal = V(0.5f, 2.5f);
    auto path = find_path(mesh, start, goal);
    ASSERT_TRUE(path.has_value());
    expect_points_eq(path->points, {start, V(1, 1), goal});
    expect_path_contained(mesh, *path);
}

TEST(FindPath, MirroredLShapeBendsExactlyAtReflexCorner)
{
    const NavMesh mesh = make_mirrored_l_mesh();
    const Vec2 start = V(0.5f, 0.5f);
    const Vec2 goal = V(1.5f, 2.5f);
    auto path = find_path(mesh, start, goal);
    ASSERT_TRUE(path.has_value());
    expect_points_eq(path->points, {start, V(1, 1), goal});
    expect_path_contained(mesh, *path);
}

TEST(FindPath, GoalAtMeshCornerBendsAtReflexCorner)
{
    const NavMesh mesh = make_l_mesh();
    const Vec2 start = V(1.5f, 0.5f);
    const Vec2 goal = V(0, 3); // exact corner of the vertical arm
    auto path = find_path(mesh, start, goal);
    ASSERT_TRUE(path.has_value());
    expect_points_eq(path->points, {start, V(1, 1), goal});
    expect_path_contained(mesh, *path);
}

// -- SIM-004: determinism ----------------------------------------------------------

TEST(FindPath, RepeatedCallsReturnIdenticalPointSequences)
{
    const NavMesh mesh = make_l_mesh();
    const Vec2 start = V(1.5f, 0.5f);
    const Vec2 goal = V(0.5f, 2.5f);
    const auto first = find_path(mesh, start, goal);
    const auto second = find_path(mesh, start, goal);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    expect_points_eq(second->points, first->points);
}

TEST(FindPath, CopiedMeshFindsIdenticalPath)
{
    const NavMesh mesh = make_l_mesh();
    const NavMesh copy = mesh; // shared immutable representation
    const Vec2 start = V(1.5f, 0.5f);
    const Vec2 goal = V(0.5f, 2.5f);
    const auto first = find_path(mesh, start, goal);
    const auto second = find_path(copy, start, goal);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    expect_points_eq(second->points, first->points);
}

} // namespace
