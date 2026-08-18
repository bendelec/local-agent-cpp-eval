#pragma once

// Private reference-only mesh representation. Never installed; only the canonical
// headers in candidate/include/vwmini/ are public. Shared by the mesh module now and
// the path module later.

#include <vwmini/nav_mesh.hpp>

#include <array>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace vwmini {

/// Sentinel for "no adjacent triangle across this edge".
inline constexpr std::size_t kNoNeighbor = std::numeric_limits<std::size_t>::max();

/// Immutable validated mesh state. `triangles` holds the accepted CCW triangles in
/// original order; `neighbors[i][e]` is the index of the triangle sharing edge `e`
/// (vertices `e` and `e+1` modulo 3) of triangle `i`, or kNoNeighbor on a boundary.
struct NavMesh::Impl {
    Impl(std::vector<Polygon> tris, std::vector<std::array<std::size_t, 3>> nb)
        : triangles(std::move(tris)), neighbors(std::move(nb))
    {
    }

    std::vector<Polygon> triangles;
    std::vector<std::array<std::size_t, 3>> neighbors;
};

} // namespace vwmini
