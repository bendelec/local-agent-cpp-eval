#include <vwmini/nav_mesh.hpp>

#include "test_util.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace {

using namespace vwmini::test;
using vwmini::ErrorCode;
using vwmini::find_path;
using vwmini::NavMesh;
using vwmini::Polygon;
using vwmini::Vec2;

// Unit square split along the (0,0)-(1,1) diagonal into two CCW triangles.
std::vector<Polygon> unit_square()
{
    return {Tri(V(0, 0), V(1, 0), V(1, 1)), Tri(V(0, 0), V(1, 1), V(0, 1))};
}

// -- MSH-004 / MSH-007: acceptance -----------------------------------------

TEST(NavMesh, ValidSquareIsAcceptedWithTwoCells)
{
    auto mesh = NavMesh::create(unit_square());
    ASSERT_TRUE(mesh.has_value());
    EXPECT_EQ(mesh->cell_count(), 2u);
}

TEST(NavMesh, CopyPreservesCellCount)
{
    auto mesh = NavMesh::create(unit_square());
    ASSERT_TRUE(mesh.has_value());
    NavMesh copy = *mesh;
    EXPECT_EQ(copy.cell_count(), 2u);
    EXPECT_TRUE(copy.contains(V(0.5f, 0.5f)));
}

TEST(NavMesh, VertexOnlyTouchIsValidWithoutAdjacency)
{
    // Two triangles meeting only at the shared vertex (1,0).
    std::vector<Polygon> tris{Tri(V(0, 0), V(1, 0), V(0, 1)), Tri(V(1, 0), V(2, 0), V(1, 1))};
    auto mesh = NavMesh::create(std::move(tris));
    ASSERT_TRUE(mesh.has_value());
    EXPECT_EQ(mesh->cell_count(), 2u);
}

TEST(NavMesh, DisjointComponentsAreValid)
{
    std::vector<Polygon> tris{Tri(V(0, 0), V(1, 0), V(0, 1)), Tri(V(5, 5), V(6, 5), V(5, 6))};
    auto mesh = NavMesh::create(std::move(tris));
    ASSERT_TRUE(mesh.has_value());
    EXPECT_EQ(mesh->cell_count(), 2u);
}

TEST(NavMesh, ThinTriangleWithinEpsilonHeightIsAccepted)
{
    // Doubled area 1e-4 exceeds the 1e-8 degeneracy bound, so the cell is valid
    // even though its apex lies within epsilon of its own base edge: a T-junction
    // requires an edge of another triangle.
    auto mesh = NavMesh::create({Tri(V(0, 0), V(1, 0), V(0.5f, 5e-5f))});
    ASSERT_TRUE(mesh.has_value());
    EXPECT_EQ(mesh->cell_count(), 1u);
    EXPECT_TRUE(mesh->contains(V(0.5f, 0.0f)));
}

// -- MSH-004: rejection matrix ---------------------------------------------

TEST(NavMesh, EmptyListIsInvalidMesh)
{
    EXPECT_EQ(code_of(NavMesh::create({})), ErrorCode::InvalidMesh);
}

TEST(NavMesh, NonFiniteVertexIsInvalidArgument)
{
    const float inf = std::numeric_limits<float>::infinity();
    auto r = NavMesh::create({Tri(V(0, 0), V(1, 0), V(inf, 1))});
    EXPECT_EQ(code_of(r), ErrorCode::InvalidArgument);
}

TEST(NavMesh, NonTrianglePolygonIsInvalidMesh)
{
    auto r = NavMesh::create({Poly({V(0, 0), V(1, 0), V(1, 1), V(0, 1)})});
    EXPECT_EQ(code_of(r), ErrorCode::InvalidMesh);
}

TEST(NavMesh, ClockwiseTriangleIsInvalidMesh)
{
    auto r = NavMesh::create({Tri(V(0, 0), V(0, 1), V(1, 0))});
    EXPECT_EQ(code_of(r), ErrorCode::InvalidMesh);
}

TEST(NavMesh, DegenerateCollinearTriangleIsInvalidMesh)
{
    auto r = NavMesh::create({Tri(V(0, 0), V(1, 1), V(2, 2))});
    EXPECT_EQ(code_of(r), ErrorCode::InvalidMesh);
}

TEST(NavMesh, OverlappingInteriorsIsInvalidMesh)
{
    // Small triangle strictly inside a large one.
    std::vector<Polygon> tris{Tri(V(0, 0), V(4, 0), V(0, 4)), Tri(V(1, 1), V(2, 1), V(1, 2))};
    EXPECT_EQ(code_of(NavMesh::create(std::move(tris))), ErrorCode::InvalidMesh);
}

TEST(NavMesh, DuplicateTriangleIsInvalidMesh)
{
    std::vector<Polygon> tris{Tri(V(0, 0), V(1, 0), V(0, 1)), Tri(V(0, 0), V(1, 0), V(0, 1))};
    EXPECT_EQ(code_of(NavMesh::create(std::move(tris))), ErrorCode::InvalidMesh);
}

TEST(NavMesh, NonManifoldEdgeIsInvalidMesh)
{
    // Edge (0,0)-(1,0) used by three triangles (t1 and t3 identical direction).
    std::vector<Polygon> tris{Tri(V(0, 0), V(1, 0), V(0.5f, 1)), Tri(V(1, 0), V(0, 0), V(0.5f, -1)),
                              Tri(V(0, 0), V(1, 0), V(0.5f, 1))};
    EXPECT_EQ(code_of(NavMesh::create(std::move(tris))), ErrorCode::InvalidMesh);
}

TEST(NavMesh, TJunctionIsInvalidMesh)
{
    // Vertex (2,0) of the small triangle lies mid-edge of the large triangle.
    std::vector<Polygon> tris{Tri(V(0, 0), V(4, 0), V(0, 4)), Tri(V(2, 0), V(2, -1), V(3, -1))};
    EXPECT_EQ(code_of(NavMesh::create(std::move(tris))), ErrorCode::InvalidMesh);
}

// -- MSH-006: containment boundary policy ----------------------------------

TEST(NavMesh, ContainsInteriorAndExterior)
{
    auto mesh = NavMesh::create(unit_square());
    ASSERT_TRUE(mesh.has_value());
    EXPECT_TRUE(mesh->contains(V(0.5f, 0.5f)));
    EXPECT_TRUE(mesh->contains(V(0.25f, 0.25f)));
    EXPECT_FALSE(mesh->contains(V(2.0f, 2.0f)));
    EXPECT_FALSE(mesh->contains(V(-1.0f, 0.5f)));
}

TEST(NavMesh, ContainsBoundaryWithinEpsilonIsTrue)
{
    auto mesh = NavMesh::create(unit_square());
    ASSERT_TRUE(mesh.has_value());
    // 5e-5 m outside edges/corner is within the 1e-4 m tolerance.
    EXPECT_TRUE(mesh->contains(V(-5e-5f, 0.5f)));       // left edge
    EXPECT_TRUE(mesh->contains(V(0.5f, -5e-5f)));       // bottom edge
    EXPECT_TRUE(mesh->contains(V(0.5f, 1.0f + 5e-5f))); // top edge
    EXPECT_TRUE(mesh->contains(V(-5e-5f, -5e-5f)));     // corner (0,0)
    EXPECT_TRUE(mesh->contains(V(-7e-5f, -7e-5f)));     // diagonal from corner: 9.9e-5 m away
    EXPECT_TRUE(mesh->contains(V(0.0f, 0.0f)));         // exact corner
}

TEST(NavMesh, ContainsBeyondEpsilonIsFalse)
{
    auto mesh = NavMesh::create(unit_square());
    ASSERT_TRUE(mesh.has_value());
    EXPECT_FALSE(mesh->contains(V(-2e-4f, 0.5f)));
    EXPECT_FALSE(mesh->contains(V(0.5f, 1.0f + 2e-4f)));
    EXPECT_FALSE(mesh->contains(V(-2e-4f, -2e-4f)));
    // Diagonally outside corner (0,0): nearest edge point is the corner itself at
    // 1.13e-4 m, beyond epsilon. MSH-006 measures segment distance, not distance
    // to the edge lines, so the inflated corner wedge does not count.
    EXPECT_FALSE(mesh->contains(V(-8e-5f, -8e-5f)));
}

TEST(NavMesh, ContainsNonFiniteIsFalse)
{
    auto mesh = NavMesh::create(unit_square());
    ASSERT_TRUE(mesh.has_value());
    const float nan = std::nanf("");
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(mesh->contains(V(nan, 0.5f)));
    EXPECT_FALSE(mesh->contains(V(0.5f, inf)));
}

// Regression guard for the moved-from NavMesh guards: a moved-from mesh must
// behave as empty (no cells, contains nothing, find_path reports OutsideMesh)
// and must not dereference the null impl.
TEST(NavMesh, MovedFromMeshBehavesAsEmpty)
{
    auto mesh = NavMesh::create(unit_square());
    ASSERT_TRUE(mesh.has_value());
    NavMesh moved = std::move(*mesh);
    EXPECT_EQ(moved.cell_count(), 2u); // the move target keeps the cells
    EXPECT_EQ(mesh->cell_count(), 0u);
    EXPECT_FALSE(mesh->contains(V(0.5f, 0.5f)));
    EXPECT_EQ(code_of(find_path(*mesh, V(0.5f, 0.5f), V(0.9f, 0.1f))), ErrorCode::OutsideMesh);
}

} // namespace
