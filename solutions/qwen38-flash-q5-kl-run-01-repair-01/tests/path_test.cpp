#include <vwmini/nav_mesh.hpp>

#include "mesh_fixtures.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace
{

using vwmini::ErrorCode;
using vwmini::NavMesh;
using vwmini::Path;
using vwmini::Vec2;

/// Walk the polyline in small strides and require every sample to be walkable (SIM-002).
void expect_segments_contained(const vwmini::NavMesh &mesh, const std::vector<vwmini::Vec2> &points)
{
    ASSERT_GE(points.size(), 2u);
    for (std::size_t index = 0; index + 1 < points.size(); ++index)
    {
        const vwmini::Vec2 from = points[index];
        const vwmini::Vec2 delta = points[index + 1] - from;
        for (float step = 0.0f; step <= 1.0f; step += 1.0e-3f)
        {
            const vwmini::Vec2 sample = from + delta * step;
            ASSERT_TRUE(mesh.contains(sample))
                << "segment " << index << " at t=" << step << " (" << sample.x << "," << sample.y
                << ") is outside the mesh";
        }
    }
}

[[nodiscard]] float polyline_length(const std::vector<vwmini::Vec2> &points)
{
    float total = 0.0f;
    for (std::size_t index = 0; index + 1 < points.size(); ++index)
    {
        total += length(points[index + 1] - points[index]);
    }
    return total;
}

/// An L: the corridor [0,3]x[0,1] plus the unit square [2,3]x[1,2].
[[nodiscard]] vwmini::NavMesh l_shaped_mesh()
{
    std::vector<fixtures::Polygon> cells = fixtures::corridor(3);
    for (fixtures::Polygon cell : fixtures::corridor(1))
    {
        for (vwmini::Vec2 &vertex : cell.vertices)
        {
            vertex = vertex + vwmini::Vec2{2.0f, 1.0f};
        }
        cells.push_back(cell);
    }
    return fixtures::make_mesh(std::move(cells));
}

/// A route of exactly two points means the straight segment itself is walkable.
[[nodiscard]] bool straight_line_is_walkable(const vwmini::NavMesh &mesh, vwmini::Vec2 from,
                                             vwmini::Vec2 to)
{
    const auto direct = vwmini::find_path(mesh, from, to);
    return direct.has_value() && direct->points.size() == 2u;
}

TEST(PathTest, RejectsNonFiniteEndpoints)
{
    const auto mesh = fixtures::make_mesh(fixtures::square_pair());
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(vwmini::find_path(mesh, Vec2{nan, 0.5f}, Vec2{1.0f, 1.0f}).error().code,
              ErrorCode::InvalidArgument);
    EXPECT_EQ(vwmini::find_path(mesh, Vec2{0.5f, 0.5f},
                                Vec2{1.0f, std::numeric_limits<float>::infinity()})
                  .error()
                  .code,
              ErrorCode::InvalidArgument);
}

TEST(PathTest, RejectsEndpointsOutsideMesh)
{
    const auto mesh = fixtures::make_mesh(fixtures::square_pair(2.0f));
    EXPECT_EQ(find_path(mesh, Vec2{5.0f, 1.0f}, Vec2{1.0f, 1.0f}).error().code,
              ErrorCode::OutsideMesh);
    EXPECT_EQ(find_path(mesh, Vec2{1.0f, 1.0f}, Vec2{5.0f, 1.0f}).error().code,
              ErrorCode::OutsideMesh);
}

TEST(PathTest, EqualEndpointsReturnSinglePoint)
{
    const auto mesh = fixtures::make_mesh(fixtures::square_pair());
    const auto path = find_path(mesh, Vec2{0.5f, 0.5f}, Vec2{0.5f, 0.5f});
    ASSERT_TRUE(path.has_value());
    ASSERT_EQ(path->points.size(), 1u);
    EXPECT_EQ(path->points[0], Vec2(0.5f, 0.5f));
}

TEST(PathTest, DirectLineIsReturnedExactly)
{
    const auto mesh = fixtures::make_mesh(fixtures::square_pair(2.0f));
    const Vec2 start{0.25f, 0.3f};
    const Vec2 goal{1.7f, 1.9f};
    const auto path = find_path(mesh, start, goal);
    ASSERT_TRUE(path.has_value());
    ASSERT_EQ(path->points.size(), 2u);
    EXPECT_EQ(path->points.front(), start);
    EXPECT_EQ(path->points.back(), goal);
}

TEST(PathTest, DisconnectedComponentsReportNoPath)
{
    const auto mesh = fixtures::make_mesh(fixtures::two_islands());
    const auto path = find_path(mesh, Vec2{0.2f, 0.2f}, Vec2{20.2f, 0.2f});
    ASSERT_FALSE(path.has_value());
    EXPECT_EQ(path.error().code, ErrorCode::NoPath);
}

/// MSH-005: triangles that touch at a single vertex are not adjacent, so no route exists.
TEST(PathTest, VertexOnlyTouchCreatesNoAdjacency)
{
    const vwmini::NavMesh mesh = fixtures::make_mesh(std::vector<fixtures::Polygon>{
        fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{0.0f, 1.0f}),
        fixtures::triangle(Vec2{0.0f, 1.0f}, Vec2{1.0f, 2.0f}, Vec2{0.0f, 2.0f})});
    EXPECT_TRUE(mesh.contains(Vec2{0.2f, 0.2f}));
    EXPECT_TRUE(mesh.contains(Vec2{0.3f, 1.5f}));
    const auto path = vwmini::find_path(mesh, Vec2{0.2f, 0.2f}, Vec2{0.3f, 1.5f});
    ASSERT_FALSE(path.has_value());
    EXPECT_EQ(path.error().code, ErrorCode::NoPath);
}

TEST(PathTest, PathAroundCornerIsContainedAndTaut)
{
    const auto mesh = l_shaped_mesh();
    const Vec2 start{0.5f, 0.5f};
    const Vec2 goal{2.5f, 1.5f};

    const auto path = find_path(mesh, start, goal);
    ASSERT_TRUE(path.has_value()) << path.error().message;
    EXPECT_EQ(path->points.front(), start);
    EXPECT_EQ(path->points.back(), goal);
    EXPECT_GT(path->points.size(), 2u);

    expect_segments_contained(mesh, path->points);

    // No waypoint may be skipped: removing any of them would cut through unwalkable space.
    for (std::size_t index = 0; index + 2 < path->points.size(); ++index)
    {
        EXPECT_FALSE(straight_line_is_walkable(mesh, path->points[index], path->points[index + 2]))
            << "waypoint " << index << " is redundant";
    }
}

TEST(PathTest, PathIsShorterThanTheCentroidRoute)
{
    const auto mesh = l_shaped_mesh();
    const auto path = find_path(mesh, Vec2{0.5f, 0.5f}, Vec2{2.5f, 1.5f});
    ASSERT_TRUE(path.has_value());
    // Straight-line distance is 2.236 m; the geodesic via the corner (2,1) is 2.288 m.
    // An unshortened route through the six cell centroids is far longer.
    EXPECT_LT(polyline_length(path->points), 2.6f);
}

TEST(PathTest, RepeatedQueriesAreIdentical)
{
    const auto mesh = l_shaped_mesh();
    const auto first = find_path(mesh, Vec2{0.4f, 0.6f}, Vec2{2.6f, 1.4f});
    const auto again = find_path(mesh, Vec2{0.4f, 0.6f}, Vec2{2.6f, 1.4f});
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(again.has_value());
    EXPECT_EQ(first->points, again->points);
}

TEST(PathTest, ConsecutiveIntermediatePointsAreNotCloserThanEpsilon)
{
    const auto mesh = l_shaped_mesh();
    const auto path = find_path(mesh, Vec2{0.5f, 0.5f}, Vec2{2.5f, 1.5f});
    ASSERT_TRUE(path.has_value());
    for (std::size_t index = 0; index + 2 < path->points.size(); ++index)
    {
        EXPECT_GT(length(path->points[index + 1] - path->points[index]), 1e-4f);
    }
}

} // namespace
