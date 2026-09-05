#include <vwmini/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

using namespace vwmini;

namespace {
double ring_area(const std::vector<Vec2>& v)
{
    const int n = static_cast<int>(v.size());
    double a = 0;
    for (int i = 0; i < n; ++i) {
        a += cross(v[i], v[(i + 1) % n]);
    }
    return 0.5 * a;
}

bool pt_in_tri_strict(Vec2 p, Vec2 a, Vec2 b, Vec2 c)
{
    return cross(b - a, p - a) > 0.0f && cross(c - b, p - b) > 0.0f && cross(a - c, p - c) > 0.0f;
}
} // namespace

TEST(Triangulate, TrianglePassesThrough)
{
    const Polygon p{{Vec2{0, 0}, {2, 0}, {0, 2}}};
    auto r = triangulate_simple_polygon(p);
    ASSERT_TRUE(r);
    ASSERT_EQ(r->size(), 1u);
    // Ear clipping may cyclically rotate the triangle; compare as a vertex set.
    ASSERT_EQ((*r)[0].vertices.size(), 3u);
    // Ear clipping may cyclically rotate the triangle; compare as a vertex set.
    const std::vector<Vec2>& got = (*r)[0].vertices;
    const std::vector<Vec2>& want = p.vertices;
    for (const Vec2& w : want) {
        bool found = false;
        for (const Vec2& g : got) {
            if (g == w) {
                found = true;
                break;
            }
        }
        EXPECT_TRUE(found);
    }
}

TEST(Triangulate, ConvexSquareYieldsTwoCCWTriangles)
{
    const Polygon square{{Vec2{0, 0}, {2, 0}, {2, 2}, {0, 2}}};
    auto r = triangulate_simple_polygon(square);
    ASSERT_TRUE(r);
    ASSERT_EQ(r->size(), 2u);
    double sum = 0;
    for (const auto& t : *r) {
        ASSERT_EQ(t.vertices.size(), 3u);
        EXPECT_GT(ring_area(t.vertices), 1e-8);
        sum += ring_area(t.vertices);
    }
    EXPECT_NEAR(sum, ring_area(square.vertices), 1e-4);
}

TEST(Triangulate, ConcaveLShapePreservesAreaAndConvexity)
{
    const Polygon lshape{{Vec2{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}}};
    auto r = triangulate_simple_polygon(lshape);
    ASSERT_TRUE(r);
    // Hexagon -> 4 triangles.
    ASSERT_EQ(r->size(), 4u);
    double sum = 0;
    for (const auto& t : *r) {
        ASSERT_EQ(t.vertices.size(), 3u);
        EXPECT_GT(ring_area(t.vertices), 1e-8);
        sum += ring_area(t.vertices);
    }
    EXPECT_NEAR(sum, ring_area(lshape.vertices), 1e-4);
}

TEST(Triangulate, OutputTrianglesDoNotOverlap)
{
    const Polygon lshape{{Vec2{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}}};
    auto r = triangulate_simple_polygon(lshape);
    ASSERT_TRUE(r);
    for (std::size_t i = 0; i < r->size(); ++i) {
        for (std::size_t j = i + 1; j < r->size(); ++j) {
            // Interiors of distinct output triangles must not share any point:
            // no vertex of one may be strictly inside the other.
            for (const Vec2& p : (*r)[i].vertices) {
                EXPECT_FALSE(pt_in_tri_strict(p, (*r)[j].vertices[0], (*r)[j].vertices[1],
                                              (*r)[j].vertices[2]));
            }
        }
    }
}

TEST(Triangulate, Deterministic)
{
    const Polygon lshape{{Vec2{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}}};
    auto a = triangulate_simple_polygon(lshape);
    auto b = triangulate_simple_polygon(lshape);
    ASSERT_TRUE(a);
    ASSERT_TRUE(b);
    ASSERT_EQ(a->size(), b->size());
    for (std::size_t i = 0; i < a->size(); ++i) {
        EXPECT_EQ((*a)[i].vertices, (*b)[i].vertices);
    }
}

TEST(Triangulate, TooFewVerticesRejected)
{
    const Polygon p{{Vec2{0, 0}, {1, 0}}};
    auto r = triangulate_simple_polygon(p);
    EXPECT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

TEST(Triangulate, ClockwiseRejected)
{
    const Polygon p{{Vec2{0, 0}, {0, 2}, {2, 0}}}; // CW
    auto r = triangulate_simple_polygon(p);
    EXPECT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

TEST(Triangulate, CollinearDegenerateRejected)
{
    const Polygon p{{Vec2{0, 0}, {1, 0}, {2, 0}}};
    auto r = triangulate_simple_polygon(p);
    EXPECT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

TEST(Triangulate, SelfIntersectingRejected)
{
    const Polygon p{{Vec2{0, 0}, {4, 1}, {4, 0}, {0, 2}}}; // bowtie, nonzero area
    auto r = triangulate_simple_polygon(p);
    EXPECT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

TEST(Triangulate, CyclicDuplicateVertexRejected)
{
    const Polygon p{{Vec2{0, 0}, {2, 0}, {2, 2}, {0, 0}}}; // first == last
    auto r = triangulate_simple_polygon(p);
    EXPECT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::InvalidMesh);
}

TEST(Triangulate, NonFiniteCoordinateRejected)
{
    const Polygon p{{Vec2{0, 0}, {2, 0}, Vec2{std::numeric_limits<float>::quiet_NaN(), 2}}};
    auto r = triangulate_simple_polygon(p);
    EXPECT_FALSE(r);
    EXPECT_EQ(r.error().code, ErrorCode::InvalidArgument);
}
