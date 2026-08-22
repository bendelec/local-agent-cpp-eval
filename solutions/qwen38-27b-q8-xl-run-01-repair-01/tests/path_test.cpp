// Path finding tests (SIM-001..SIM-004): validation, direct paths, corridor
// string-pulling, containment of every output segment, and determinism.
#include <vwmini/nav_mesh.hpp>

#include "test_util.hpp"

#include <cmath>
#include <limits>
#include <vector>

using namespace vwmini;
using vwmini::testing::err_code;
using vwmini::testing::tri;

namespace {

[[nodiscard]] float nan_f()
{
    return std::numeric_limits<float>::quiet_NaN();
}

// Unit square [0,1]^2 split along the main diagonal (0,0)-(1,1).
NavMesh unitSquare()
{
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {1, 1}), tri({0, 0}, {1, 1}, {0, 1})});
    EXPECT_TRUE(r.has_value());
    return *r;
}

// Frame: [0,4]^2 minus the blocked square [1,3]^2 (16 right-triangle cells,
// integer lattice, no T-junctions). The walkable space is a U-shaped ring.
NavMesh uShape()
{
    std::vector<Polygon> t;
    auto add = [&t](Vec2 a, Vec2 b, Vec2 c) { t.push_back(tri(a, b, c)); };
    // Bottom row.
    add({0, 0}, {1, 0}, {1, 1});
    add({0, 0}, {1, 1}, {0, 1});
    add({1, 0}, {3, 0}, {3, 1});
    add({1, 0}, {3, 1}, {1, 1});
    add({3, 0}, {4, 0}, {4, 1});
    add({3, 0}, {4, 1}, {3, 1});
    // Middle row (the two side passages around the blocked square).
    add({0, 1}, {1, 1}, {1, 3});
    add({0, 1}, {1, 3}, {0, 3});
    add({3, 1}, {4, 1}, {4, 3});
    add({3, 1}, {4, 3}, {3, 3});
    // Top row.
    add({0, 3}, {1, 3}, {1, 4});
    add({0, 3}, {1, 4}, {0, 4});
    add({1, 3}, {3, 3}, {3, 4});
    add({1, 3}, {3, 4}, {1, 4});
    add({3, 3}, {4, 3}, {4, 4});
    add({3, 3}, {4, 4}, {3, 4});
    auto r = NavMesh::create(std::move(t));
    EXPECT_TRUE(r.has_value());
    return *r;
}

// Independent coarse containment cross-check (not the library's exact test):
// dense sampling of every output segment.
bool sampledContained(const NavMesh& mesh, const Path& path)
{
    for (std::size_t i = 1; i < path.points.size(); ++i)
    {
        const Vec2 a = path.points[i - 1];
        const Vec2 b = path.points[i];
        for (int k = 0; k <= 200; ++k)
        {
            const float t = float(k) / 200.f;
            if (!mesh.contains(a + (b - a) * t))
                return false;
        }
    }
    return true;
}

float pathLength(const Path& path)
{
    float total = 0.f;
    for (std::size_t i = 1; i < path.points.size(); ++i)
        total += vwmini::length(path.points[i] - path.points[i - 1]);
    return total;
}

} // namespace

TEST(PathValidation, NonFiniteEndpointsReturnInvalidArgument)
{
    const NavMesh mesh = unitSquare();
    const Vec2 p = {0.5f, 0.5f};
    ASSERT_FALSE(find_path(mesh, {nan_f(), 0.f}, p));
    EXPECT_EQ(err_code(find_path(mesh, {nan_f(), 0.f}, p)), ErrorCode::InvalidArgument);
    EXPECT_EQ(err_code(find_path(mesh, p, {std::numeric_limits<float>::infinity(), 0.f})),
              ErrorCode::InvalidArgument);
    EXPECT_EQ(err_code(find_path(mesh, p, {-std::numeric_limits<float>::infinity(),
                                           std::numeric_limits<float>::infinity()})),
              ErrorCode::InvalidArgument);
}

TEST(PathValidation, OutsideMeshEndpoints)
{
    const NavMesh mesh = unitSquare();
    EXPECT_EQ(err_code(find_path(mesh, {2.f, 0.5f}, {0.5f, 0.5f})),
              ErrorCode::OutsideMesh);
    EXPECT_EQ(err_code(find_path(mesh, {0.5f, 0.5f}, {-1.f, -1.f})),
              ErrorCode::OutsideMesh);
}

TEST(PathValidation, DisconnectedComponentsReturnNoPath)
{
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {0, 1}),
                              tri({10, 10}, {11, 10}, {10, 11})});
    ASSERT_TRUE(r.has_value());
    const NavMesh mesh = *r;
    EXPECT_EQ(err_code(find_path(mesh, {0.25f, 0.25f}, {10.25f, 10.25f})),
              ErrorCode::NoPath);
}

TEST(DirectPath, EqualEndpointsReturnSinglePoint)
{
    const NavMesh mesh = unitSquare();
    const Vec2 p = {0.4f, 0.3f};
    auto result = find_path(mesh, p, p);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->points.size(), 1u);
    EXPECT_EQ(result->points[0], p);
}

TEST(DirectPath, StraightSegmentInOneCell)
{
    const NavMesh mesh = unitSquare();
    const Vec2 start = {0.6f, 0.1f};
    const Vec2 goal = {0.8f, 0.2f};
    auto result = find_path(mesh, start, goal);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->points.size(), 2u);
    EXPECT_EQ(result->points[0], start);
    EXPECT_EQ(result->points[1], goal);
}

TEST(DirectPath, StraightSegmentAcrossCells)
{
    // The segment crosses the shared diagonal: still exactly [start, goal].
    const NavMesh mesh = unitSquare();
    const Vec2 start = {0.2f, 0.2f};
    const Vec2 goal = {0.8f, 0.3f};
    auto result = find_path(mesh, start, goal);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->points.size(), 2u);
    EXPECT_EQ(result->points[0], start);
    EXPECT_EQ(result->points[1], goal);
}

TEST(DirectPath, StartOnSharedEdge)
{
    const NavMesh mesh = unitSquare();
    const Vec2 start = {0.5f, 0.5f}; // on the diagonal
    const Vec2 goal = {0.9f, 0.1f};
    auto result = find_path(mesh, start, goal);
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->points.size(), 2u);
    EXPECT_EQ(result->points[0], start);
    EXPECT_EQ(result->points[1], goal);
}

TEST(UShape, PathHugsBlockedCorner)
{
    const NavMesh mesh = uShape();
    const Vec2 start = {2.f, 0.25f};
    const Vec2 goal = {2.f, 3.25f};
    auto result = find_path(mesh, start, goal);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    const Path& path = *result;

    // Endpoints are preserved (SIM-002).
    ASSERT_EQ(path.points.size(), 4u);
    EXPECT_EQ(path.points[0], start);
    EXPECT_EQ(path.points[3], goal);

    // The optimum hugs one of the two symmetric inner corners:
    // (1,1)-(1,3) or (3,1)-(3,3).
    const bool isLeft = path.points[1] == (Vec2{1.f, 1.f}) &&
                        path.points[2] == (Vec2{1.f, 3.f});
    const bool isRight = path.points[1] == (Vec2{3.f, 1.f}) &&
                         path.points[2] == (Vec2{3.f, 3.f});
    EXPECT_TRUE(isLeft || isRight);

    // Expected length: 1.25 + 2 + sqrt(1.0625).
    EXPECT_NEAR(pathLength(path), 1.25f + 2.f + std::sqrt(1.0625f), 1e-4f);

    // SIM-002: consecutive points no closer than epsilon.
    for (std::size_t i = 1; i < path.points.size(); ++i)
        EXPECT_GT(vwmini::length(path.points[i] - path.points[i - 1]),
                  vwmini::testing::kEps);

    // Independent coarse containment cross-check.
    EXPECT_TRUE(sampledContained(mesh, path));
}

TEST(UShape, DeterminismAcrossCalls)
{
    const NavMesh mesh = uShape();
    const Vec2 start = {2.f, 0.25f};
    const Vec2 goal = {2.f, 3.25f};
    auto first = find_path(mesh, start, goal);
    ASSERT_TRUE(first.has_value());
    for (int i = 0; i < 3; ++i)
    {
        auto again = find_path(mesh, start, goal);
        ASSERT_TRUE(again.has_value());
        EXPECT_EQ(again->points, first->points);
    }
    // Same mesh, different mesh object (value semantics): identical result.
    const NavMesh copy = mesh;
    auto fromCopy = find_path(copy, start, goal);
    ASSERT_TRUE(fromCopy.has_value());
    EXPECT_EQ(fromCopy->points, first->points);
}

TEST(UShape, RouteFromBoundaryPoint)
{
    const NavMesh mesh = uShape();
    // Start on the shared edge between the bottom-middle and bottom-left
    // cells; the straight segment to the goal is blocked by the square.
    const Vec2 start = {1.f, 0.5f};
    const Vec2 goal = {1.5f, 3.5f};
    auto result = find_path(mesh, start, goal);
    ASSERT_TRUE(result.has_value()) << result.error().message;
    const Path& path = *result;
    ASSERT_GE(path.points.size(), 2u);
    EXPECT_EQ(path.points.front(), start);
    EXPECT_EQ(path.points.back(), goal);
    EXPECT_TRUE(sampledContained(mesh, path));
}
