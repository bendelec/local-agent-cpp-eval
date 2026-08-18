// Geometry track (G): Vec2 vector semantics (MSH-001), outline validation and error
// codes (MSH-002), and deterministic triangulation (MSH-003).

#include "test_fixtures.hpp"

#include <vwmini/geometry.hpp>

#include <cmath>
#include <vector>

using namespace vwmini;
using namespace vwmini_test;

namespace {

[[nodiscard]] Polygon ring(std::vector<Vec2> vertices)
{
    return Polygon{std::move(vertices)};
}

} // namespace

// ---- MSH-001: Vec2 value-semantic vector operations ----

TEST(Geometry_Vec2, ExactEquality)
{
    EXPECT_TRUE((Vec2{1.0f, 2.0f} == Vec2{1.0f, 2.0f}));
    EXPECT_FALSE((Vec2{1.0f, 2.0f} == Vec2{1.0f, 2.0001f}));
    EXPECT_FALSE((Vec2{1.0f, 2.0f} == Vec2{1.0001f, 2.0f}));
}

TEST(Geometry_Vec2, Arithmetic)
{
    EXPECT_EQ((Vec2{1.0f, 2.0f} + Vec2{3.0f, 4.0f}), (Vec2{4.0f, 6.0f}));
    EXPECT_EQ((Vec2{3.0f, 4.0f} - Vec2{1.0f, 2.0f}), (Vec2{2.0f, 2.0f}));
    EXPECT_EQ((Vec2{1.0f, 2.0f} * 3.0f), (Vec2{3.0f, 6.0f}));
    EXPECT_EQ((3.0f * Vec2{1.0f, 2.0f}), (Vec2{3.0f, 6.0f}));
}

TEST(Geometry_Vec2, DotAndCross)
{
    EXPECT_EQ(dot(Vec2{2.0f, 3.0f}, Vec2{4.0f, 5.0f}), 23.0f);
    EXPECT_EQ(cross(Vec2{2.0f, 3.0f}, Vec2{4.0f, 5.0f}), -2.0f);
}

TEST(Geometry_Vec2, LengthAndNormalized)
{
    EXPECT_NEAR(length(Vec2{3.0f, 4.0f}), 5.0f, 1e-5f);
    EXPECT_NEAR(length(Vec2{-3.0f, 4.0f}), 5.0f, 1e-5f);
    EXPECT_NEAR(length(Vec2{0.0f, 0.0f}), 0.0f, 1e-6f);

    const Vec2 unit = normalized(Vec2{3.0f, 4.0f});
    EXPECT_NEAR(unit.x, 0.6f, 1e-5f);
    EXPECT_NEAR(unit.y, 0.8f, 1e-5f);

    // Normalizing the zero vector yields the zero vector (MSH-001).
    EXPECT_EQ(normalized(Vec2{0.0f, 0.0f}), (Vec2{0.0f, 0.0f}));
}

// ---- MSH-002: outline validation and error codes ----

TEST(Geometry_Triangulate, NonFiniteCoordinateIsInvalidArgument)
{
    expect_error(triangulate_simple_polygon(
                     ring({{0.0f, 0.0f}, {10.0f, 0.0f}, {std::nanf(""), 5.0f}})),
                 ErrorCode::InvalidArgument);
    expect_error(triangulate_simple_polygon(
                     ring({{0.0f, 0.0f}, {10.0f, 0.0f}, {5.0f, INFINITY}})),
                 ErrorCode::InvalidArgument);
}

TEST(Geometry_Triangulate, TooFewVerticesIsInvalidMesh)
{
    expect_error(triangulate_simple_polygon(ring({})), ErrorCode::InvalidMesh);
    expect_error(triangulate_simple_polygon(ring({{0.0f, 0.0f}})), ErrorCode::InvalidMesh);
    expect_error(triangulate_simple_polygon(ring({{0.0f, 0.0f}, {10.0f, 0.0f}})),
                 ErrorCode::InvalidMesh);
}

TEST(Geometry_Triangulate, ConsecutiveDuplicateVerticesAreInvalidMesh)
{
    // Adjacent duplicates.
    expect_error(triangulate_simple_polygon(
                     ring({{0.0f, 0.0f}, {0.0f, 0.0f}, {10.0f, 0.0f}, {0.0f, 10.0f}})),
                 ErrorCode::InvalidMesh);
    // Equal first and last (cyclic wrap).
    expect_error(triangulate_simple_polygon(
                     ring({{0.0f, 0.0f}, {10.0f, 0.0f}, {0.0f, 10.0f}, {0.0f, 0.0f}})),
                 ErrorCode::InvalidMesh);
}

TEST(Geometry_Triangulate, ClockwiseWindingIsInvalidMesh)
{
    expect_error(triangulate_simple_polygon(
                     ring({{0.0f, 0.0f}, {0.0f, 10.0f}, {10.0f, 0.0f}})),
                 ErrorCode::InvalidMesh);
}

TEST(Geometry_Triangulate, DegenerateAreaIsInvalidMesh)
{
    expect_error(triangulate_simple_polygon(
                     ring({{0.0f, 0.0f}, {5.0f, 0.0f}, {10.0f, 0.0f}})),
                 ErrorCode::InvalidMesh);
}

TEST(Geometry_Triangulate, SelfIntersectionIsInvalidMesh)
{
    // Bowtie: edges (0,0)-(10,10) and (10,0)-(0,10) cross.
    expect_error(triangulate_simple_polygon(
                     ring({{0.0f, 0.0f}, {10.0f, 10.0f}, {10.0f, 0.0f}, {0.0f, 10.0f}})),
                 ErrorCode::InvalidMesh);
}

TEST(Geometry_Triangulate, CollinearOutlineVertexIsPermitted)
{
    // A square with an extra collinear point on the bottom edge is valid.
    const auto result = triangulate_simple_polygon(
        ring({{0.0f, 0.0f}, {5.0f, 0.0f}, {10.0f, 0.0f}, {10.0f, 10.0f}, {0.0f, 10.0f}}));
    ASSERT_TRUE(result.has_value());
    // The collinear vertex is dropped, leaving a quadrilateral -> two triangles.
    EXPECT_EQ(result->size(), 2u);
}

// ---- MSH-003: deterministic, non-degenerate, CCW triangles ----

TEST(Geometry_Triangulate, SquareProducesTwoTrianglesWithCorrectArea)
{
    const Polygon square = ring({{0.0f, 0.0f}, {10.0f, 0.0f}, {10.0f, 10.0f}, {0.0f, 10.0f}});
    const auto result = triangulate_simple_polygon(square);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2u);

    double area_sum = 0.0;
    for (const Polygon& tri : *result) {
        EXPECT_EQ(tri.vertices.size(), 3u);
        const double a = signed_area(tri);
        EXPECT_GT(a, 0.0); // every triangle is CCW and non-degenerate
        area_sum += a;
    }
    const double input_area = signed_area(square);
    EXPECT_NEAR(area_sum, input_area, (kEps * std::max(1.0, input_area)));
}

TEST(Geometry_Triangulate, PentagonProducesThreeTriangles)
{
    const auto result = triangulate_simple_polygon(
        ring({{0.0f, 0.0f}, {4.0f, 0.0f}, {5.0f, 3.0f}, {2.0f, 5.0f}, {-1.0f, 2.0f}}));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 3u);
}

TEST(Geometry_Triangulate, EveryTriangleVertexIsAnInputVertex)
{
    const Polygon hex = ring({{0.0f, 0.0f}, {4.0f, 0.0f}, {6.0f, 3.0f}, {4.0f, 6.0f},
                               {0.0f, 6.0f}, {-2.0f, 3.0f}});
    const auto result = triangulate_simple_polygon(hex);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 4u);

    for (const Polygon& tri : *result) {
        for (const Vec2 v : tri.vertices) {
            bool found = false;
            for (const Vec2 input : hex.vertices) {
                if (v == input) {
                    found = true;
                    break;
                }
            }
            EXPECT_TRUE(found) << "triangle vertex not present in the input outline";
        }
    }
}

TEST(Geometry_Triangulate, DeterministicAcrossRepeatedCalls)
{
    const Polygon hex = ring({{0.0f, 0.0f}, {4.0f, 0.0f}, {6.0f, 3.0f}, {4.0f, 6.0f},
                              {0.0f, 6.0f}, {-2.0f, 3.0f}});
    const auto first = triangulate_simple_polygon(hex);
    const auto second = triangulate_simple_polygon(hex);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    ASSERT_EQ(first->size(), second->size());
    for (std::size_t i = 0; i < first->size(); ++i) {
        EXPECT_EQ((*first)[i].vertices, (*second)[i].vertices);
    }
}
