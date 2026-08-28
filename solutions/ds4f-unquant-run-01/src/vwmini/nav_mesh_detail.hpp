#pragma once

// Private definition of NavMesh::Impl, shared between nav_mesh.cpp and path.cpp.

#include <vwmini/nav_mesh.hpp>

#include <array>
#include <cstdint>
#include <vector>

namespace vwmini {

struct NavMesh::Impl {
    // Accepted triangles in input order, with original float coordinates.
    std::vector<Polygon> triangles;
    // Unique mesh points (exact-equality dedup of input vertices).
    std::vector<Vec2> points;
    // Per triangle, the point indices of its three vertices.
    std::vector<std::array<std::size_t, 3>> verts;
    // Per triangle, the neighbour triangle index for each edge, or -1 when the
    // edge is unmatched (boundary, crack, or T-junction side).
    std::vector<std::array<std::int32_t, 3>> neighbours;
};

} // namespace vwmini
