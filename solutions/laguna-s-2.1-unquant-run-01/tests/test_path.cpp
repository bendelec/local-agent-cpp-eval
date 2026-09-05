#include <vwmini/nav_mesh.hpp>

#include "test_common.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace vwmini;

namespace {
Polygon tri(Vec2 a, Vec2 b, Vec2 c) { return Polygon{{a, b, c}}; }

// Path must start at start and end at goal.
bool ends_at(const Path& p, Vec2 start, Vec2 goal)
{
    if (p.points.empty()) {
        return false;
    }
    return length(p.points.front() - start) <= 1e-4f &&
           length(p.points.back() - goal) <= 1e-4f;
}
} // namespace

TEST(FindPath, SameCellIsDirectSegment)
{
    const NavMesh m = test::make_square();
    const Vec2 s{0.5f, 0.5f};
    const Vec2 g{1.5f, 0.5f};
    auto r = find_path(m, s, g);
    ASSERT_TRUE(r);
    ASSERT_EQ(r->points.size(), 2u);
    EXPECT_EQ(r->points[0], s);
    EXPECT_EQ(r->points[1], g);
}

TEST(FindPath, StartEqualsGoalIsSingleton)
{
    const NavMesh m = test::make_square();
    const Vec2 s{0.5f, 0.5f};
    auto r = find_path(m, s, s);
    ASSERT_TRUE(r);
    ASSERT_EQ(r->points.size(), 1u);
    EXPECT_EQ(r->points[0], s);
}

TEST(FindPath, EndpointOutsideMeshIsOutsideMesh)
{
    const NavMesh m = test::make_square();
    auto r = find_path(m, Vec2{0.5f, 0.5f}, Vec2{3.0f, 3.0f});
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::OutsideMesh);
}

TEST(FindPath, NonFiniteInputIsInvalidArgument)
{
    const NavMesh m = test::make_square();
    auto r = find_path(m, Vec2{std::numeric_limits<float>::quiet_NaN(), 0.0f}, Vec2{1.0f, 1.0f});
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
}

TEST(FindPath, DiagonalPathCrossesSharedEdgeAndIsContained)
{
    const NavMesh m = test::make_square();
    const Vec2 s{0.1f, 0.1f};
    const Vec2 g{1.9f, 1.9f};
    auto r = find_path(m, s, g);
    ASSERT_TRUE(r);
    EXPECT_TRUE(ends_at(*r, s, g));
    EXPECT_TRUE(test::path_contained(m, *r));
    EXPECT_LT(test::path_length(*r), 2.6f); // optimal ~2.546 (straight diagonal)
}

TEST(FindPath, GridCorridorStaysContainedAndConnectsEndpoints)
{
    const NavMesh m = test::make_grid();
    const Vec2 s{0.1f, 0.1f};
    const Vec2 g{3.9f, 3.9f};
    auto r = find_path(m, s, g);
    ASSERT_TRUE(r);
    EXPECT_TRUE(ends_at(*r, s, g));
    EXPECT_TRUE(test::path_contained(m, *r));
}

TEST(FindPath, Deterministic)
{
    const NavMesh m = test::make_grid();
    const Vec2 s{0.1f, 0.1f};
    const Vec2 g{3.9f, 3.9f};
    auto a = find_path(m, s, g);
    auto b = find_path(m, s, g);
    ASSERT_TRUE(a);
    ASSERT_TRUE(b);
    EXPECT_EQ(a->points, b->points);
}

TEST(FindPath, DisconnectedMeshYieldsNoPath)
{
    // Two disjoint triangles (allowed by MSH-005) but unreachable from one to the other.
    auto m = NavMesh::create({tri({0, 0}, {1, 0}, {1, 1}), tri({5, 5}, {6, 5}, {6, 6})});
    ASSERT_TRUE(m);
    auto r = find_path(*m, Vec2{0.1f, 0.1f}, Vec2{5.5f, 5.5f});
    ASSERT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::NoPath);
}
