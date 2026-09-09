#pragma once

// Definition of the opaque `NavMesh::Impl` declared in the public header, together
// with the aliases for its representation types and the small helpers over them.
// Kept in an internal header so both `nav_mesh.cpp` (construction/containment) and
// `pathfinding.cpp` (a friend of `NavMesh`) can share the representation without
// exposing it publicly. The mesh is immutable once built (MSH-007).

#include <vwmini/nav_mesh.hpp>

#include "vwmini/internal/predicates.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace vwmini::internal {

/// One accepted triangle: three CCW, non-degenerate corners in caller order.
using Cell = std::array<Vec2, 3>;
/// A triple of cell or welded-vertex indices; -1 means "none". See `NavMesh::Impl`.
using Index3 = std::array<std::int32_t, 3>;

/// Directed edge `k` of a cell runs from corner `k` to corner `next_corner(k)`.
[[nodiscard]] constexpr std::size_t next_corner(std::size_t k) noexcept
{
    return (k + 1) % 3;
}

/// True when `cell` contains `point` under the MSH-006 boundary policy.
[[nodiscard]] inline bool cell_contains(Vec2 point, const Cell& cell) noexcept
{
    return triangle_contains_eps(point, cell[0], cell[1], cell[2]);
}

} // namespace vwmini::internal

namespace vwmini {

struct NavMesh::Impl {
    /// Accepted triangles in caller order, each CCW and non-degenerate.
    std::vector<internal::Cell> cells;
    /// `adjacency[i][k]` is the cell sharing cell `i`'s edge `k` (from `cells[i][k]`
    /// to `cells[i][(k+1)%3]`), or -1 when that edge is on the mesh boundary.
    std::vector<internal::Index3> adjacency;
};

} // namespace vwmini
