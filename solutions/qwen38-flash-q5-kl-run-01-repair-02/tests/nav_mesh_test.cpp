#include <vwmini/nav_mesh.hpp>

#include "mesh_fixtures.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace
{

using vwmini::ErrorCode;
using vwmini::NavMesh;
using vwmini::Vec2;

[[nodiscard]] ErrorCode failure_code_of(const std::vector<fixtures::Polygon> &cells)
{
    const auto result = NavMesh::create(cells);
    EXPECT_FALSE(result.has_value());
    return result.error().code;
}

TEST(NavMeshTest, AcceptsTwoAdjacentTriangles)
{
    const auto mesh = NavMesh::create(fixtures::square_pair());
    ASSERT_TRUE(mesh.has_value());
    EXPECT_EQ(mesh->cell_count(), 2u);
}

TEST(NavMeshTest, EmptyTriangleListIsInvalidMesh)
{
    EXPECT_EQ(vwmini::NavMesh::create({}).error().code, ErrorCode::InvalidMesh);
}

TEST(NavMeshTest, RejectsQuadCell)
{
    const fixtures::Polygon quad{
        {Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{1.0f, 1.0f}, Vec2{0.0f, 1.0f}}};
    EXPECT_EQ(failure_code_of({quad}), ErrorCode::InvalidMesh);
}

TEST(NavMeshTest, RejectsClockwiseTriangle)
{
    EXPECT_EQ(
        failure_code_of({fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{0.0f, 1.0f}, Vec2{1.0f, 0.0f})}),
        ErrorCode::InvalidMesh);
}

TEST(NavMeshTest, RejectsDegenerateTriangle)
{
    EXPECT_EQ(
        failure_code_of({fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{2.0f, 0.0f})}),
        ErrorCode::InvalidMesh);
}

TEST(NavMeshTest, RejectsTriangleWithRepeatedVertex)
{
    EXPECT_EQ(
        failure_code_of({fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{0.0f, 0.0f})}),
        ErrorCode::InvalidMesh);
}

TEST(NavMeshTest, RejectsNonFiniteCoordinate)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(
        failure_code_of({fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{nan, 1.0f})}),
        ErrorCode::InvalidArgument);
    EXPECT_EQ(
        failure_code_of({fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f},
                                            Vec2{1.0f, std::numeric_limits<float>::infinity()})}),
        ErrorCode::InvalidArgument);
}

TEST(NavMeshTest, RejectsOverlappingInteriors)
{
    const auto first = fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f}, Vec2{0.0f, 2.0f});
    const auto overlap = fixtures::triangle(Vec2{0.5f, 0.5f}, Vec2{3.0f, 0.5f}, Vec2{0.5f, 3.0f});
    EXPECT_EQ(failure_code_of({first, overlap}), ErrorCode::InvalidMesh);
}

TEST(NavMeshTest, RejectsCrossingTriangles)
{
    const auto flat = fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}, Vec2{2.0f, 1.0f});
    const auto cross = fixtures::triangle(Vec2{0.0f, 1.0f}, Vec2{4.0f, 1.0f}, Vec2{2.0f, -1.0f});
    EXPECT_EQ(failure_code_of({flat, cross}), ErrorCode::InvalidMesh);
}

TEST(NavMeshTest, RejectsDuplicateTriangle)
{
    const auto cell = fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{0.0f, 1.0f});
    EXPECT_EQ(failure_code_of({cell, cell}), ErrorCode::InvalidMesh);
}

TEST(NavMeshTest, RejectsNonManifoldEdge)
{
    // Three triangles around the edge (0,0)-(1,0): three cells share one edge.
    const auto a = fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{0.5f, 1.0f});
    const auto b = fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{0.5f, -1.0f}, Vec2{1.0f, 0.0f});
    const auto c = fixtures::triangle(Vec2{1.0f, 0.0f}, Vec2{0.0f, 0.0f}, Vec2{1.5f, -1.0f});
    EXPECT_EQ(failure_code_of({a, b, c}), ErrorCode::InvalidMesh);
}

TEST(NavMeshTest, RejectsTJunction)
{
    // Vertex (1,0) of the second triangle rests in the middle of the first triangle's edge.
    const auto base = fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{2.0f, 0.0f}, Vec2{0.0f, 2.0f});
    const auto touching = fixtures::triangle(Vec2{1.0f, 0.0f}, Vec2{2.0f, -1.0f}, Vec2{2.0f, 0.5f});
    EXPECT_EQ(failure_code_of({base, touching}), ErrorCode::InvalidMesh);
}

TEST(NavMeshTest, AllowsDisjointComponents)
{
    const auto mesh = NavMesh::create(fixtures::two_islands());
    ASSERT_TRUE(mesh.has_value());
    EXPECT_EQ(mesh->cell_count(), 2u);
}

TEST(NavMeshTest, AllowsVertexOnlyTouch)
{
    // Two triangles meeting in a single shared corner: allowed, simply not adjacent.
    const auto lower = fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{0.0f, 1.0f});
    const auto upper = fixtures::triangle(Vec2{1.0f, 0.0f}, Vec2{2.0f, 1.0f}, Vec2{1.0f, 1.0f});
    const auto mesh = NavMesh::create({lower, upper});
    ASSERT_TRUE(mesh.has_value());
    EXPECT_EQ(mesh->cell_count(), 2u);
}

TEST(NavMeshTest, ContainsInteriorExteriorAndBoundaryBand)
{
    const auto mesh = fixtures::make_mesh(fixtures::square_pair(2.0f));

    EXPECT_TRUE(mesh.contains(Vec2{1.0f, 1.0f}));
    EXPECT_FALSE(mesh.contains(Vec2{3.0f, 1.0f}));
    EXPECT_FALSE(mesh.contains(Vec2{-1.0f, 3.0f}));

    // On a closed edge, and just outside it but still inside the epsilon band.
    EXPECT_TRUE(mesh.contains(Vec2{1.0f, 0.0f}));
    EXPECT_TRUE(mesh.contains(Vec2{1.0f, -0.5f * 1e-4f}));
    EXPECT_TRUE(mesh.contains(Vec2{2.0f + 0.5f * 1e-4f, 1.0f}));
    EXPECT_TRUE(mesh.contains(Vec2{1.0f, 2.0f + 0.9f * 1e-4f}));
    // Beyond the epsilon band the same points are outside.
    EXPECT_FALSE(mesh.contains(Vec2{1.0f, -3.0f * 1e-4f}));
    EXPECT_FALSE(mesh.contains(Vec2{2.0f + 3.0f * 1e-4f, 1.0f}));
    EXPECT_FALSE(mesh.contains(Vec2{1.0f, 2.0f + 3.0f * 1e-4f}));

    // Corners are within the band of two edges.
    EXPECT_TRUE(mesh.contains(Vec2{2.0f, 2.0f}));
    EXPECT_FALSE(mesh.contains(Vec2{2.0f + 2.0e-3f, 2.0f + 2.0e-3f}));
}

TEST(NavMeshTest, ContainsRejectsNonFinitePoints)
{
    const auto mesh = fixtures::make_mesh(fixtures::square_pair());
    EXPECT_FALSE(mesh.contains(Vec2{std::numeric_limits<float>::quiet_NaN(), 0.5f}));
    EXPECT_FALSE(mesh.contains(Vec2{0.5f, std::numeric_limits<float>::infinity()}));
}

TEST(NavMeshTest, MeshValuesShareTheSameImmutableImpl)
{
    const auto mesh = fixtures::make_mesh(fixtures::square_pair());
    const auto copy = mesh;

    EXPECT_EQ(mesh.cell_count(), copy.cell_count());
    EXPECT_TRUE(copy.contains(Vec2{0.5f, 0.5f}));
}

TEST(NavMeshTest, FailedCreateLeavesExistingMeshUsable)
{
    const auto mesh = fixtures::make_mesh(fixtures::corridor(3));
    const std::size_t cells_before = mesh.cell_count();

    EXPECT_EQ(NavMesh::create({}).error().code, ErrorCode::InvalidMesh);
    EXPECT_EQ(
        NavMesh::create({fixtures::triangle(Vec2{0.0f, 0.0f}, Vec2{0.0f, 1.0f}, Vec2{1.0f, 0.0f})})
            .error()
            .code,
        ErrorCode::InvalidMesh);

    EXPECT_EQ(mesh.cell_count(), cells_before);
    EXPECT_TRUE(mesh.contains(Vec2{0.5f, 0.5f}));
}

} // namespace
