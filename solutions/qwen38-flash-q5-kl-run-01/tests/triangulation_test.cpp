#include <vwmini/geometry.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{

using vwmini::ErrorCode;
using vwmini::Polygon;
using vwmini::Vec2;

Polygon unit_square(float size = 2.0f)
{
    return Polygon{{Vec2{0.0f, 0.0f}, Vec2{size, 0.0f}, Vec2{size, size}, Vec2{0.0f, size}}};
}

float signed_area(const Polygon &triangle)
{
    return vwmini::cross(triangle.vertices[1] - triangle.vertices[0],
                         triangle.vertices[2] - triangle.vertices[0]) *
           0.5f;
}

float total_area(const std::vector<Polygon> &triangles)
{
    float area = 0.0f;
    for (const Polygon &triangle : triangles)
    {
        area += signed_area(triangle);
    }
    return area;
}

bool all_counter_clockwise(const std::vector<Polygon> &triangles)
{
    return std::all_of(triangles.begin(), triangles.end(),
                       [](const Polygon &t) { return signed_area(t) > 0.0f; });
}

bool uses_only_input_vertices(const std::vector<Polygon> &triangles,
                              const std::vector<Vec2> &inputs)
{
    for (const Polygon &triangle : triangles)
    {
        for (const Vec2 &vertex : triangle.vertices)
        {
            const bool found = std::any_of(inputs.begin(), inputs.end(), [vertex](Vec2 candidate)
                                           { return candidate == vertex; });
            if (!found)
            {
                return false;
            }
        }
    }
    return true;
}

TEST(TriangulateTest, SquareProducesTwoCcwTriangles)
{
    const auto result = vwmini::triangulate_simple_polygon(unit_square());
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2u);
    EXPECT_TRUE(all_counter_clockwise(*result));
}

TEST(TriangulateTest, AreaMatchesInputWithinTolerance)
{
    const auto result = vwmini::triangulate_simple_polygon(unit_square(3.0f));
    ASSERT_TRUE(result.has_value());
    const float input_area = 9.0f;
    EXPECT_NEAR(total_area(*result), input_area, 1e-4f * std::max(1.0f, input_area));
}

TEST(TriangulateTest, OutputIsDeterministic)
{
    const Polygon outline{{Vec2{0, 0}, Vec2{4, 0}, Vec2{4, 1}, Vec2{2, 3}, Vec2{0, 2}}};
    const auto first = vwmini::triangulate_simple_polygon(outline);
    const auto again = vwmini::triangulate_simple_polygon(outline);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(again.has_value());
    ASSERT_EQ(first->size(), again->size());
    for (std::size_t i = 0; i < first->size(); ++i)
    {
        EXPECT_EQ((*first)[i].vertices, (*again)[i].vertices);
    }
}

TEST(TriangulateTest, EveryTriangleVertexComesFromTheOutline)
{
    const Polygon outline{{Vec2{0, 0}, Vec2{4, 0}, Vec2{4, 1}, Vec2{2, 3}, Vec2{0, 2}}};
    const auto result = vwmini::triangulate_simple_polygon(outline);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(uses_only_input_vertices(*result, outline.vertices));
}

TEST(TriangulateTest, ConcaveOutlineTriangulatesWithoutOverlap)
{
    // An arrow-head outline with one reflex vertex: a naive fan would overlap.
    const Polygon outline{
        {Vec2{0.0f, 0.0f}, Vec2{4.0f, 0.0f}, Vec2{1.5f, 1.0f}, Vec2{4.0f, 4.0f}, Vec2{0.0f, 4.0f}}};
    const auto result = vwmini::triangulate_simple_polygon(outline);
    ASSERT_TRUE(result.has_value());
    // Equal areas imply no overlap: all triangles are CCW, so overlapping interiors would
    // make the sum strictly larger than the outline area.
    EXPECT_NEAR(total_area(*result), 11.0f, 1e-4f);
    EXPECT_TRUE(all_counter_clockwise(*result));
}

TEST(TriangulateTest, CollinearOutlineVertexIsAllowed)
{
    Polygon outline = unit_square();
    outline.vertices.insert(outline.vertices.begin() + 1, Vec2{1.0f, 0.0f});
    const auto result = vwmini::triangulate_simple_polygon(outline);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(total_area(*result), 4.0f, 1e-4f);
}

TEST(TriangulateTest, NonFiniteCoordinateIsInvalidArgument)
{
    Polygon outline = unit_square();
    outline.vertices[1] = Vec2{std::numeric_limits<float>::infinity(), 1.0f};
    const auto result = vwmini::triangulate_simple_polygon(outline);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::InvalidArgument);
}

TEST(TriangulateTest, TooFewVerticesIsInvalidMesh)
{
    EXPECT_EQ(vwmini::triangulate_simple_polygon(Polygon{}).error().code, ErrorCode::InvalidMesh);
    EXPECT_EQ(vwmini::triangulate_simple_polygon(Polygon{{Vec2{0, 0}, Vec2{1, 1}}}).error().code,
              ErrorCode::InvalidMesh);
}

TEST(TriangulateTest, ClosedOutlineIsRejectedAsDuplicate)
{
    Polygon outline = unit_square();
    outline.vertices.push_back(outline.vertices.front());
    EXPECT_EQ(vwmini::triangulate_simple_polygon(outline).error().code, ErrorCode::InvalidMesh);
}

TEST(TriangulateTest, ConsecutiveDuplicateVertexIsRejected)
{
    Polygon outline = unit_square();
    outline.vertices.insert(outline.vertices.begin() + 1, outline.vertices[0]);
    EXPECT_EQ(vwmini::triangulate_simple_polygon(outline).error().code, ErrorCode::InvalidMesh);
}

TEST(TriangulateTest, ClockwiseOutlineIsRejected)
{
    const Polygon clockwise{{Vec2{0, 0}, Vec2{0, 2}, Vec2{2, 2}, Vec2{2, 0}}};
    EXPECT_EQ(vwmini::triangulate_simple_polygon(clockwise).error().code, ErrorCode::InvalidMesh);
}

TEST(TriangulateTest, DegenerateAreaIsRejected)
{
    const Polygon degenerate{{Vec2{0, 0}, Vec2{1, 0}, Vec2{2, 0}}};
    EXPECT_EQ(vwmini::triangulate_simple_polygon(degenerate).error().code, ErrorCode::InvalidMesh);
}

TEST(TriangulateTest, SelfIntersectingOutlineIsRejected)
{
    // Edges 0->1 and 2->3 cross at (1,1).
    const Polygon bowtie{{Vec2{0, 0}, Vec2{2, 2}, Vec2{2, 0}, Vec2{0, 2}}};
    EXPECT_EQ(vwmini::triangulate_simple_polygon(bowtie).error().code, ErrorCode::InvalidMesh);
}

} // namespace
