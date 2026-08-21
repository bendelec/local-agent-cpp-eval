#include <vwmini/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "test_util.hpp"

using vwmini::ErrorCode;
using vwmini::Polygon;
using vwmini::Result;
using vwmini::triangulate_simple_polygon;
namespace vt = vwmini::testing;

namespace {

Polygon outline(std::initializer_list<vwmini::Vec2> pts)
{
    return Polygon{std::vector<vwmini::Vec2>{pts}};
}

double area(const Polygon& p) { return vt::signed_area_sum({p}); }

void ExpectMesh(const Polygon& poly)
{
    const Result<std::vector<Polygon>> r = triangulate_simple_polygon(poly);
    ASSERT_TRUE(r.has_value()) << r.error().message;
    const std::vector<Polygon>& tris = *r;
    EXPECT_EQ(tris.size(), poly.vertices.size() - 2);
    EXPECT_TRUE(vt::triangulation_valid(poly, tris)) << "invalid triangulation";
    const double sum = vt::signed_area_sum(tris);
    const double a = area(poly);
    EXPECT_NEAR(sum, a, 1e-4 * std::max(1.0, std::abs(a))) << "area not conserved";
}

} // namespace

// MSH-002: non-finite coordinates -> InvalidArgument
TEST(Triangulate, NonFiniteCoordinates)
{
    const float nanf = std::numeric_limits<float>::quiet_NaN();
    const Result<std::vector<Polygon>> r =
        triangulate_simple_polygon(outline({{0, 0}, {1, nanf}, {1, 1}, {0, 1}}));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
}

// MSH-002: fewer than three vertices -> InvalidMesh
TEST(Triangulate, TooFewVertices)
{
    const Polygon polys[] = {outline({}), outline({{0, 0}}), outline({{0, 0}, {1, 0}})};
    for (const Polygon& p : polys)
    {
        const Result<std::vector<Polygon>> r = triangulate_simple_polygon(p);
        ASSERT_FALSE(r.has_value());
        EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
    }
}

// MSH-002: cyclic consecutive duplicate vertices (including first == last)
TEST(Triangulate, ConsecutiveDuplicates)
{
    const Polygon polys[] = {outline({{0, 0}, {0, 0}, {1, 0}, {0, 1}}),
                             outline({{0, 0}, {1, 0}, {0, 1}, {0, 0}})};
    for (const Polygon& p : polys)
    {
        const Result<std::vector<Polygon>> r = triangulate_simple_polygon(p);
        ASSERT_FALSE(r.has_value());
        EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
    }
}

// MSH-002: self-intersection (bowtie)
TEST(Triangulate, SelfIntersection)
{
    const Result<std::vector<Polygon>> r =
        triangulate_simple_polygon(outline({{0, 0}, {2, 2}, {2, 0}, {0, 2}}));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

// MSH-002: boundary self-touch (vertex on non-adjacent edge / spike)
TEST(Triangulate, SelfTouch)
{
    // v2 sits on edge v0-v1.
    const Result<std::vector<Polygon>> spike =
        triangulate_simple_polygon(outline({{0, 0}, {2, 0}, {1, 0}, {0, 1}}));
    ASSERT_FALSE(spike.has_value());
    EXPECT_EQ(spike.error().code, ErrorCode::InvalidMesh);

    // Two triangles meeting only at a repeated non-adjacent vertex.
    const Result<std::vector<Polygon>> touch = triangulate_simple_polygon(
        outline({{1, 1}, {2, 1}, {2, 2}, {1, 1}, {0, 2}, {0, 1}}));
    ASSERT_FALSE(touch.has_value());
    EXPECT_EQ(touch.error().code, ErrorCode::InvalidMesh);
}

// MSH-002: clockwise winding
TEST(Triangulate, ClockwiseWinding)
{
    const Result<std::vector<Polygon>> r =
        triangulate_simple_polygon(outline({{0, 0}, {0, 1}, {1, 0}}));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

// MSH-002: degenerate area (collinear)
TEST(Triangulate, DegenerateArea)
{
    const Result<std::vector<Polygon>> r =
        triangulate_simple_polygon(outline({{0, 0}, {1, 0}, {2, 0}}));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

// MSH-002: collinear non-duplicate outline vertices are permitted.
TEST(Triangulate, CollinearVertexAccepted)
{
    ExpectMesh(outline({{0, 0}, {1, 0}, {2, 0}, {2, 2}, {0, 2}}));
}

// MSH-003: convex quadrilateral
TEST(Triangulate, ConvexQuadrilateral)
{
    ExpectMesh(outline({{0, 0}, {4, 0}, {4, 3}, {1, 2}}));
}

// MSH-003: convex pentagon
TEST(Triangulate, ConvexPentagon)
{
    ExpectMesh(outline({{0, 0}, {5, 0}, {6, 3}, {3, 5}, {0, 4}}));
}

// MSH-003: concave L-shape (6 vertices -> 4 triangles)
TEST(Triangulate, ConcaveLShape)
{
    ExpectMesh(outline({{0, 0}, {2, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 2}}));
}

// MSH-003: deeply concave arrow (dart)
TEST(Triangulate, ConcaveArrow)
{
    ExpectMesh(outline({{0, 0}, {6, 3}, {0, 6}, {2, 3}}));
}

// MSH-003: identical input produces identical ordered output.
TEST(Triangulate, Deterministic)
{
    const Polygon poly =
        outline({{0, 0}, {4, 0}, {5, 2}, {4, 4}, {2, 5}, {0, 4}, {-1, 2}});
    const auto r1 = triangulate_simple_polygon(poly);
    const auto r2 = triangulate_simple_polygon(poly);
    ASSERT_TRUE(r1.has_value());
    ASSERT_TRUE(r2.has_value());
    ASSERT_EQ(r1->size(), r2->size());
    for (std::size_t i = 0; i < r1->size(); ++i)
        for (std::size_t k = 0; k < 3; ++k)
            EXPECT_EQ((*r1)[i].vertices[k], (*r2)[i].vertices[k]);
}
