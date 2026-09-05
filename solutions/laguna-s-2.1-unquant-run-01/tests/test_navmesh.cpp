#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <vector>

using namespace vwmini;

namespace {
Polygon tri(Vec2 a, Vec2 b, Vec2 c) { return Polygon{{a, b, c}}; }

// A valid 2-triangle square mesh used across tests.
std::vector<Polygon> square_polys()
{
    return {tri({0, 0}, {2, 0}, {2, 2}), tri({0, 0}, {2, 2}, {0, 2})};
}

TEST(NavMeshCreate, ValidSquareMesh)
{
    auto m = NavMesh::create(square_polys());
    ASSERT_TRUE(m);
    EXPECT_EQ(m->cell_count(), 2u);
}

TEST(NavMeshCreate, EmptyInputRejected)
{
    auto m = NavMesh::create({});
    EXPECT_FALSE(m);
    EXPECT_EQ(m.error().code, ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, TooSmallOrTooLargePolygonsRejected)
{
    // Two-vertex polygon.
    auto m1 = NavMesh::create({tri({0, 0}, {1, 0}, {1, 1}),
                               Polygon{{Vec2{0, 0}, {1, 0}}}});
    EXPECT_FALSE(m1);
    EXPECT_EQ(m1.error().code, ErrorCode::InvalidMesh);

    // Four-vertex polygon.
    auto m2 = NavMesh::create({Polygon{{Vec2{0, 0}, {1, 0}, {1, 1}, {0, 1}}}});
    EXPECT_FALSE(m2);
    EXPECT_EQ(m2.error().code, ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, ClockwiseTriangleRejected)
{
    // CW winding of the square's first triangle.
    auto m = NavMesh::create({tri({0, 0}, {2, 2}, {2, 0}), tri({0, 0}, {2, 2}, {0, 2})});
    EXPECT_FALSE(m);
    EXPECT_EQ(m.error().code, ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, DegenerateTriangleRejected)
{
    auto m = NavMesh::create({tri({0, 0}, {2, 0}, {4, 0})}); // collinear
    EXPECT_FALSE(m);
    EXPECT_EQ(m.error().code, ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, NonManifoldEdgeRejected)
{
    // Three triangles sharing edge (0,0)-(2,2): non-manifold.
    auto m = NavMesh::create({
        tri({0, 0}, {2, 0}, {2, 2}),
        tri({0, 0}, {2, 2}, {0, 2}),
        tri({0, 0}, {2, 2}, {4, 2})});
    EXPECT_FALSE(m);
    EXPECT_EQ(m.error().code, ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, OverlappingInteriorsRejected)
{
    // Two triangles with overlapping interiors (no shared edge).
    auto m = NavMesh::create({
        tri({0, 0}, {4, 0}, {4, 4}),
        tri({1, 1}, {5, 1}, {1, 5})});
    EXPECT_FALSE(m);
    EXPECT_EQ(m.error().code, ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, TJunctionRejected)
{
    // Left triangle's vertex (1,0) lies in the interior of the right edge.
    auto m = NavMesh::create({
        tri({0, 0}, {1, 0}, {1, 1}),
        tri({1, 0}, {2, 0}, {2, 1})});
    EXPECT_FALSE(m);
    EXPECT_EQ(m.error().code, ErrorCode::InvalidMesh);
}

TEST(NavMeshContains, StrictlyInsideAndEdges)
{
    auto m = NavMesh::create(square_polys());
    ASSERT_TRUE(m);
    EXPECT_TRUE(m->contains(Vec2{0.5f, 0.5f}));
    EXPECT_TRUE(m->contains(Vec2{1.9f, 1.9f}));
    // On the shared diagonal edge.
    EXPECT_TRUE(m->contains(Vec2{1.0f, 1.0f}));
    // On a boundary edge.
    EXPECT_TRUE(m->contains(Vec2{1.0f, 0.0f}));
    // Clearly outside.
    EXPECT_FALSE(m->contains(Vec2{3.0f, 3.0f}));
}

TEST(NavMeshContains, NonFiniteIsFalse)
{
    auto m = NavMesh::create(square_polys());
    ASSERT_TRUE(m);
    EXPECT_FALSE(m->contains(Vec2{std::numeric_limits<float>::quiet_NaN(), 0.0f}));
    EXPECT_FALSE(m->contains(Vec2{0.0f, std::numeric_limits<float>::infinity()}));
}
} // namespace
