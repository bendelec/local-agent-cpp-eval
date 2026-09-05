#pragma once

#include <vwmini/geometry.hpp>
#include <vwmini/nav_mesh.hpp>
#include <vwmini/simulation.hpp>

#include <cassert>
#include <vector>

namespace vwmini::test {

// A square (0,0)-(2,2) split into two CCW triangles along the diagonal.
inline NavMesh make_square()
{
    auto r = NavMesh::create({
        Polygon{{{Vec2{0.0f, 0.0f}, {2.0f, 0.0f}, {2.0f, 2.0f}}}},
        Polygon{{{Vec2{0.0f, 0.0f}, {2.0f, 2.0f}, {0.0f, 2.0f}}}},
    });
    assert(r);
    return *std::move(r);
}

// A 2x2 grid (0,0)-(4,4) of four cells, each split into two CCW triangles (8 tris).
inline NavMesh make_grid()
{
    auto r = NavMesh::create({
        {{Vec2{0, 0}, {2, 0}, {2, 2}}},   // cell (0,0)
        {{Vec2{0, 0}, {2, 2}, {0, 2}}},
        {{Vec2{2, 0}, {4, 0}, {4, 2}}},   // cell (2,0)
        {{Vec2{2, 0}, {4, 2}, {2, 2}}},
        {{Vec2{0, 2}, {2, 2}, {2, 4}}},   // cell (0,2)
        {{Vec2{0, 2}, {2, 4}, {0, 4}}},
        {{Vec2{2, 2}, {4, 2}, {4, 4}}},   // cell (2,2)
        {{Vec2{2, 2}, {4, 4}, {2, 4}}},
    });
    assert(r);
    return *std::move(r);
}

// A single large CCW triangle: an open convex cell for avoidance tests.
inline NavMesh make_big_triangle()
{
    auto r = NavMesh::create({
        Polygon{{{Vec2{0.0f, 0.0f}, {10.0f, 0.0f}, {0.0f, 10.0f}}}},
    });
    assert(r);
    return *std::move(r);
}

// A 20x20 right triangle providing room for crossing-avoidance tests.
inline NavMesh make_large_triangle()
{
    auto r = NavMesh::create({
        Polygon{{{Vec2{0.0f, 0.0f}, {20.0f, 0.0f}, {0.0f, 20.0f}}}},
    });
    assert(r);
    return *std::move(r);
}

// True when every sample of segment [a,b] is contained by the mesh.
inline bool segment_contained(const NavMesh& m, Vec2 a, Vec2 b, int steps = 30)
{
    for (int k = 0; k <= steps; ++k) {
        const float t = static_cast<float>(k) / static_cast<float>(steps);
        const Vec2 p = a + (b - a) * t;
        if (!m.contains(p)) {
            return false;
        }
    }
    return true;
}

// True when every segment of the path is contained in the mesh.
inline bool path_contained(const NavMesh& m, const Path& p)
{
    for (std::size_t i = 1; i < p.points.size(); ++i) {
        if (!segment_contained(m, p.points[i - 1], p.points[i])) {
            return false;
        }
    }
    return true;
}

// Euclidean length of a path polyline.
inline float path_length(const Path& p)
{
    float total = 0.0f;
    for (std::size_t i = 1; i < p.points.size(); ++i) {
        total += length(p.points[i] - p.points[i - 1]);
    }
    return total;
}

} // namespace vwmini::test
