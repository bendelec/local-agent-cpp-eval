#include <vwmini/geometry.hpp>

#include "test_util.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

using vwmini::ErrorCode;
using vwmini::Polygon;
using vwmini::Vec2;
using vwmini::test::Poly;
using vwmini::test::V;

double signed_double_area(const std::vector<Vec2>& v)
{
    double s = 0.0;
    const std::size_t n = v.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = v[i];
        const Vec2 b = v[(i + 1) % n];
        s += double(a.x) * double(b.y) - double(b.x) * double(a.y);
    }
    return s;
}

double triangle_double_area(const Polygon& t)
{
    return signed_double_area(t.vertices);
}

// -- MSH-002 validation -----------------------------------------------------

TEST(Triangulation, NonFiniteCoordinateIsInvalidArgument)
{
    const float nan = std::nanf("");
    auto r = vwmini::triangulate_simple_polygon(Poly({V(0, 0), V(1, 0), V(nan, 1)}));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
}

TEST(Triangulation, FewerThanThreeVerticesIsInvalidMesh)
{
    auto r = vwmini::triangulate_simple_polygon(Poly({V(0, 0), V(1, 1)}));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

TEST(Triangulation, ConsecutiveDuplicateIsInvalidMesh)
{
    auto r = vwmini::triangulate_simple_polygon(Poly({V(0, 0), V(1, 0), V(1, 0), V(0, 1)}));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

TEST(Triangulation, ClosedRingFirstEqualsLastIsInvalidMesh)
{
    auto r = vwmini::triangulate_simple_polygon(Poly({V(0, 0), V(1, 0), V(0, 1), V(0, 0)}));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

TEST(Triangulation, ClockwiseWindingIsInvalidMesh)
{
    auto r = vwmini::triangulate_simple_polygon(Poly({V(0, 0), V(0, 1), V(1, 0)}));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

TEST(Triangulation, DegenerateCollinearAreaIsInvalidMesh)
{
    auto r = vwmini::triangulate_simple_polygon(Poly({V(0, 0), V(1, 1), V(2, 2)}));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

TEST(Triangulation, SelfIntersectionIsInvalidMesh)
{
    // Bowtie: edges (0->1) and (2->3) cross.
    auto r = vwmini::triangulate_simple_polygon(Poly({V(0, 0), V(2, 2), V(2, 0), V(0, 2)}));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

// -- MSH-002 acceptance of collinear, MSH-003 output properties -------------

TEST(Triangulation, SquareProducesTwoCcwTrianglesPreservingArea)
{
    auto r = vwmini::triangulate_simple_polygon(Poly({V(0, 0), V(2, 0), V(2, 2), V(0, 2)}));
    ASSERT_TRUE(r.has_value());
    const std::vector<Polygon>& tris = *r;
    EXPECT_EQ(tris.size(), 2u);
    double total = 0.0;
    for (const Polygon& t : tris) {
        ASSERT_EQ(t.vertices.size(), 3u);
        const double a = triangle_double_area(t);
        EXPECT_GT(a, 1e-8) << "triangle must be CCW and non-degenerate";
        total += a;
    }
    const double input = signed_double_area({V(0, 0), V(2, 0), V(2, 2), V(0, 2)});
    EXPECT_NEAR(total, input, 1e-4 * std::max(1.0, std::abs(input)));
}

TEST(Triangulation, CollinearVertexIsAllowedAndDropped)
{
    // Bottom edge has an extra collinear vertex at (1,0).
    auto r =
        vwmini::triangulate_simple_polygon(Poly({V(0, 0), V(1, 0), V(2, 0), V(2, 2), V(0, 2)}));
    ASSERT_TRUE(r.has_value());
    // The collinear vertex (1,0) is dropped, leaving a 4-gon -> 2 triangles.
    EXPECT_EQ(r->size(), 2u);
    double total = 0.0;
    for (const Polygon& t : *r) {
        EXPECT_GT(triangle_double_area(t), 1e-8);
        total += triangle_double_area(t);
    }
    EXPECT_NEAR(total, 8.0, 1e-3); // 2x2 square double area = 8
}

TEST(Triangulation, EveryTriangleVertexIsAnInputVertex)
{
    const std::vector<Vec2> input{V(0, 0), V(3, 0), V(3, 1), V(1, 1), V(1, 3), V(0, 3)};
    auto r = vwmini::triangulate_simple_polygon(Poly(input));
    ASSERT_TRUE(r.has_value());
    for (const Polygon& t : *r) {
        for (const Vec2 p : t.vertices) {
            EXPECT_NE(std::ranges::find(input, p), input.end())
                << "triangle vertex not present in input";
        }
    }
}

TEST(Triangulation, DeterministicForIdenticalInput)
{
    const Polygon p = Poly({V(0, 0), V(4, 0), V(4, 3), V(2, 1), V(0, 3)});
    auto a = vwmini::triangulate_simple_polygon(p);
    auto b = vwmini::triangulate_simple_polygon(p);
    ASSERT_TRUE(a.has_value() && b.has_value());
    ASSERT_EQ(a->size(), b->size());
    for (std::size_t i = 0; i < a->size(); ++i) {
        EXPECT_EQ((*a)[i].vertices, (*b)[i].vertices);
    }
}

} // namespace
