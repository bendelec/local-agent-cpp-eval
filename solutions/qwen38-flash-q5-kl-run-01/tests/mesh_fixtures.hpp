#pragma once

// Small mesh builders shared by the test suites. Meshes are small and axis-aligned so the
// expected geometry stays obvious by inspection.

#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>

#include <gtest/gtest.h>

#include <utility>
#include <vector>

namespace fixtures
{

using vwmini::NavMesh;
using vwmini::Polygon;
using vwmini::Vec2;

[[nodiscard]] inline Polygon triangle(Vec2 a, Vec2 b, Vec2 c)
{
    return Polygon{{a, b, c}};
}

/// Two counter-clockwise triangles covering [0,size] x [0,size].
[[nodiscard]] inline std::vector<Polygon> square_pair(float size = 2.0f)
{
    const Vec2 origin{0.0f, 0.0f};
    const Vec2 right{size, 0.0f};
    const Vec2 far{size, size};
    const Vec2 up{0.0f, size};
    return {triangle(origin, right, far), triangle(origin, far, up)};
}

/// Straight corridor of `segments` unit squares, `width` metres tall, starting at the origin.
[[nodiscard]] inline std::vector<Polygon> corridor(std::size_t segments, float width = 1.0f)
{
    std::vector<Polygon> cells;
    for (std::size_t index = 0; index < segments; ++index)
    {
        const float left = static_cast<float>(index);
        const float right = left + 1.0f;
        const Vec2 bottom_left{left, 0.0f};
        const Vec2 bottom_right{right, 0.0f};
        const Vec2 top_right{right, width};
        const Vec2 top_left{left, width};
        cells.push_back(triangle(bottom_left, bottom_right, top_right));
        cells.push_back(triangle(bottom_left, top_right, top_left));
    }
    return cells;
}

/// L-shaped corridor: [0,2] x [0,1] joined to [1,2] x [0,3] by four unit squares.
[[nodiscard]] inline std::vector<Polygon> l_corridor()
{
    std::vector<Polygon> cells;
    for (const Vec2 &bottom_left :
         {Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{1.0f, 1.0f}, Vec2{1.0f, 2.0f}})
    {
        const Vec2 bottom_right = bottom_left + Vec2{1.0f, 0.0f};
        const Vec2 top_right = bottom_left + Vec2{1.0f, 1.0f};
        const Vec2 top_left = bottom_left + Vec2{0.0f, 1.0f};
        cells.push_back(triangle(bottom_left, bottom_right, top_right));
        cells.push_back(triangle(bottom_left, top_right, top_left));
    }
    return cells;
}

/// Two disjoint single-triangle islands far apart.
[[nodiscard]] inline std::vector<Polygon> two_islands()
{
    return {triangle(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{0.0f, 1.0f}),
            triangle(Vec2{20.0f, 0.0f}, Vec2{21.0f, 0.0f}, Vec2{20.0f, 1.0f})};
}

/// Builds a mesh and fails the calling test if the contract rejects it.
[[nodiscard]] inline NavMesh make_mesh(std::vector<Polygon> cells)
{
    auto result = NavMesh::create(std::move(cells));
    if (!result.has_value())
    {
        ADD_FAILURE() << "fixture mesh was rejected: " << result.error().message;
        return NavMesh::create({triangle(Vec2{0.0f, 0.0f}, Vec2{1.0f, 0.0f}, Vec2{0.0f, 1.0f})})
            .value();
    }
    return std::move(*result);
}

} // namespace fixtures
