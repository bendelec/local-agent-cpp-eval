#include <vwmini/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace {
// Signed area of a ring via the public cross product.
float ring_area(const std::vector<vwmini::Vec2>& v)
{
    const int n = static_cast<int>(v.size());
    float a = 0.0f;
    for (int i = 0; i < n; ++i) {
        a += cross(v[i], v[(i + 1) % n]);
    }
    return 0.5f * a;
}

bool is_convex_ccw(const std::vector<vwmini::Vec2>& v)
{
    const int n = static_cast<int>(v.size());
    if (ring_area(v) <= 1e-8f) {
        return false;
    }
    bool saw = false;
    for (int i = 0; i < n; ++i) {
        const vwmini::Vec2 a = v[i];
        const vwmini::Vec2 b = v[(i + 1) % n];
        const vwmini::Vec2 c = v[(i + 2) % n];
        const float cr = cross(b - a, c - b);
        if (cr < -1e-4f) {
            return false;
        }
        if (cr > 1e-4f) {
            saw = true;
        }
    }
    return saw;
}
} // namespace

namespace {
TEST(Vec2Ops, BasicArithmetic)
{
    const vwmini::Vec2 a{1.0f, 2.0f};
    const vwmini::Vec2 b{3.0f, -1.0f};
    EXPECT_EQ(a + b, (vwmini::Vec2{4.0f, 1.0f}));
    EXPECT_EQ(a - b, (vwmini::Vec2{-2.0f, 3.0f}));
    EXPECT_EQ(a * 2.0f, (vwmini::Vec2{2.0f, 4.0f}));
    EXPECT_EQ(2.0f * a, (vwmini::Vec2{2.0f, 4.0f}));
}

TEST(Vec2Ops, DotCrossAndLength)
{
    const vwmini::Vec2 a{3.0f, 4.0f};
    EXPECT_FLOAT_EQ(dot(a, a), 25.0f);
    EXPECT_FLOAT_EQ(length(a), 5.0f);
    EXPECT_FLOAT_EQ(cross(a, a), 0.0f);
    EXPECT_FLOAT_EQ(cross(vwmini::Vec2{1, 0}, vwmini::Vec2{0, 1}), 1.0f);
}

TEST(Vec2Ops, Normalized)
{
    const vwmini::Vec2 a{3.0f, 4.0f};
    const vwmini::Vec2 n = normalized(a);
    EXPECT_FLOAT_EQ(n.x, 0.6f);
    EXPECT_FLOAT_EQ(n.y, 0.8f);
    EXPECT_FLOAT_EQ(length(n), 1.0f);
    EXPECT_EQ(normalized(vwmini::Vec2{}), (vwmini::Vec2{0.0f, 0.0f}));
}

TEST(Triangulate, SquareYieldsTwoTrianglesWithPositiveArea)
{
    const std::vector<vwmini::Vec2> square = {
        {0, 0}, {2, 0}, {2, 2}, {0, 2}};
    auto tris = vwmini::triangulate_simple_polygon(vwmini::Polygon{square});
    ASSERT_TRUE(tris);
    ASSERT_EQ(tris->size(), 2u);
    for (const auto& t : *tris) {
        ASSERT_EQ(t.vertices.size(), 3u);
        EXPECT_GT(ring_area(t.vertices), 0.0f);
    }
}

TEST(Triangulate, TriangulationPreservesArea)
{
    const std::vector<vwmini::Vec2> shape = {
        {0, 0}, {4, 0}, {4, 3}, {2, 2}, {0, 3}};
    const auto tris = vwmini::triangulate_simple_polygon(vwmini::Polygon{shape});
    ASSERT_TRUE(tris);
    double sum = 0;
    for (const auto& t : *tris) {
        sum += ring_area(t.vertices);
    }
    EXPECT_NEAR(sum, ring_area(shape), 1e-3);
}

TEST(Triangulate, ConvexPolygonIsConvex)
{
    const std::vector<vwmini::Vec2> pentagon = {
        {0, 0}, {2, 0}, {3, 1}, {1, 2}, {-1, 1}};
    EXPECT_TRUE(is_convex_ccw(pentagon));
}
} // namespace
