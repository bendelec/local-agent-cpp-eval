#include <vwmini/nav_mesh.hpp>

#include "test_util.hpp"

#include <cmath>
#include <limits>

using namespace vwmini;
using vwmini::testing::err_code;
using vwmini::testing::tri;

namespace {

// Unit square [0,1]^2 split along the main diagonal (0,0)-(1,1).
std::vector<Polygon> square_triangles()
{
    return {tri({0, 0}, {1, 0}, {1, 1}), tri({0, 0}, {1, 1}, {0, 1})};
}

} // namespace

// --- MSH-004: create validation -------------------------------------------

TEST(NavMeshCreate, EmptyListIsInvalidMesh)
{
    auto r = NavMesh::create({});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(err_code(r), ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, NonTrianglePolygonIsInvalidMesh)
{
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {0, 1}),
                              Polygon{{Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}, Vec2{0.5f, 0.5f}}}});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(err_code(r), ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, ClockwiseTriangleIsInvalidMesh)
{
    auto r = NavMesh::create({tri({0, 0}, {0, 1}, {1, 0})}); // CW
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(err_code(r), ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, DegenerateTriangleIsInvalidMesh)
{
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {2, 0})}); // collinear
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(err_code(r), ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, SubToleranceTriangleIsInvalidMesh)
{
    // Signed double area 5e-9 is below epsilon^2 = 1e-8.
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {0, 5e-9f})});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(err_code(r), ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, NonFiniteVertexIsInvalidArgument)
{
    auto inf = std::numeric_limits<float>::infinity();
    auto nan = std::numeric_limits<float>::quiet_NaN();
    auto r = NavMesh::create({tri({inf, 0}, {1, 0}, {0, 1})});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(err_code(r), ErrorCode::InvalidArgument);
    r = NavMesh::create({tri({0, 0}, {1, 0}, {0, nan})});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(err_code(r), ErrorCode::InvalidArgument);
}

TEST(NavMeshCreate, OverlappingInteriorsAreInvalidMesh)
{
    // Two right triangles whose interiors overlap near the shared corner.
    auto r = NavMesh::create({tri({0, 0}, {2, 0}, {0, 2}), tri({1, 0}, {3, 0}, {1, 2})});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(err_code(r), ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, DuplicateTriangleIsInvalidMesh)
{
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {0, 1}), tri({0, 0}, {1, 0}, {0, 1})});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(err_code(r), ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, TjunctionIsInvalidMesh)
{
    // Vertex (1,0) sits in the open middle of the first triangle's edge
    // (0,0)-(2,0); the second triangle is on the opposite side, so there is
    // no interior overlap -- only the T-junction.
    auto r = NavMesh::create({tri({0, 0}, {2, 0}, {1, 2}), tri({2, 0}, {1, 0}, {1.5f, -1})});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(err_code(r), ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, VertexOnOpenEdgeOfNeighbourIsTjunction)
    // A vertex of one triangle strictly inside an edge of another, on the
    // same side of that edge.
{
    auto r = NavMesh::create({tri({0, 0}, {2, 0}, {1, 2}), tri({1, 0}, {2, 0}, {1, 1})});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(err_code(r), ErrorCode::InvalidMesh);
}

TEST(NavMeshCreate, ValidMeshIsAccepted)
{
    auto r = NavMesh::create(square_triangles());
    ASSERT_TRUE(r.has_value()) << r.error().message;
    EXPECT_EQ(r->cell_count(), 2u);
}

// --- MSH-005 / MSH-007 -----------------------------------------------------

TEST(NavMeshCreate, TransactionalOnFailure)
{
    // The third triangle overlaps the first; a failing create exposes no
    // partial mesh (the only observable is the error Result).
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {0, 1}), tri({0, 5}, {1, 5}, {0.5f, 6}),
                              tri({0.2f, 0.2f}, {0.8f, 0.2f}, {0.2f, 0.8f})});
    EXPECT_FALSE(r.has_value());
}

TEST(NavMesh, CopyAndMovePreserveObservables)
{
    auto r = NavMesh::create(square_triangles());
    ASSERT_TRUE(r.has_value());
    NavMesh copy = *r;
    NavMesh moved = std::move(*r);
    EXPECT_EQ(copy.cell_count(), 2u);
    EXPECT_EQ(moved.cell_count(), 2u);
    EXPECT_TRUE(copy.contains({0.25f, 0.25f}));
    EXPECT_TRUE(moved.contains({0.75f, 0.25f}));
}

// --- MSH-006: contains -----------------------------------------------------

TEST(NavMeshContains, StrictInterior)
{
    auto r = NavMesh::create(square_triangles());
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(r->contains({0.25f, 0.25f})); // lower-left triangle
    EXPECT_TRUE(r->contains({0.75f, 0.75f})); // upper-right triangle
    EXPECT_TRUE(r->contains({0.5f, 0.5f}));   // on the shared diagonal
    EXPECT_TRUE(r->contains({0.0f, 0.0f}));   // shared vertex
}

TEST(NavMeshContains, FarOutsideIsFalse)
{
    auto r = NavMesh::create(square_triangles());
    ASSERT_TRUE(r.has_value());
    EXPECT_FALSE(r->contains({2.0f, 2.0f}));
    EXPECT_FALSE(r->contains({-1.0f, 0.5f}));
    EXPECT_FALSE(r->contains({0.5f, -0.5f}));
}

TEST(NavMeshContains, BoundaryToleranceAdmitsNearEdgePoints)
{
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {0, 1})});
    ASSERT_TRUE(r.has_value());
    // Just below the bottom edge (0,0)-(1,0): admitted.
    EXPECT_TRUE(r->contains({0.5f, -1e-5f}));
    // Farther out than epsilon: not admitted.
    EXPECT_FALSE(r->contains({0.5f, -1e-3f}));
    // Just outside the hypotenuse x+y=1 (perpendicular offset 5e-5*sqrt(2)
    // ~ 7.1e-5 < epsilon): admitted.
    EXPECT_TRUE(r->contains({0.50005f, 0.50005f}));
    // Perpendicular offset 1e-4*sqrt(2) ~ 1.4e-4 > epsilon: not admitted.
    EXPECT_FALSE(r->contains({0.5001f, 0.5001f}));
}

TEST(NavMeshContains, NonFinitePointIsFalse)
{
    auto r = NavMesh::create(square_triangles());
    ASSERT_TRUE(r.has_value());
    auto inf = std::numeric_limits<float>::infinity();
    auto nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(r->contains({inf, 0.5f}));
    EXPECT_FALSE(r->contains({0.5f, nan}));
    EXPECT_FALSE(r->contains({-inf, inf}));
}

TEST(NavMeshContains, DisjointComponents)
{
    // Two separated triangles: a point in the gap is outside.
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {0, 1}),
                              tri({10, 0}, {11, 0}, {10, 1})});
    ASSERT_TRUE(r.has_value()) << r.error().message;
    EXPECT_EQ(r->cell_count(), 2u);
    EXPECT_TRUE(r->contains({0.5f, 0.2f}));
    EXPECT_TRUE(r->contains({10.5f, 0.2f}));
    EXPECT_FALSE(r->contains({5.0f, 0.5f}));
}

TEST(NavMeshContains, VertexOnlyTouchIsAllowed)
{
    // Two triangles touching only at the origin (opposite quadrants).
    auto r = NavMesh::create({tri({0, 0}, {1, 0}, {0, 1}),
                              tri({0, 0}, {-1, 0}, {0, -1})});
    ASSERT_TRUE(r.has_value()) << r.error().message;
    // The shared vertex is contained; the wedge between the triangles is not.
    EXPECT_TRUE(r->contains({0.0f, 0.0f}));
    EXPECT_FALSE(r->contains({0.2f, -0.2f}));
    EXPECT_FALSE(r->contains({-0.2f, 0.2f}));
}
