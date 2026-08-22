// Private mesh internals: triangle validation (MSH-004), shared-edge adjacency
// (MSH-005), and containment (MSH-006). `NavMesh::Impl` (private member of the
// public class) wraps `MeshImpl`; it is completed here so translation units
// with friend access to `NavMesh` can use it.
#pragma once

#include <vwmini/nav_mesh.hpp>

#include "vwmini/geometry_priv.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace vwmini::detail {

// One accepted CCW triangle plus its mesh neighbours.
struct TriCell {
    Vec2 a{}, b{}, c{};
    // (neighbour cell index, shared complete edge endpoints, in this cell's
    // own CCW edge direction). Vertex-only touches create no entry.
    std::vector<std::pair<std::size_t, std::array<Vec2, 2>>> neighbors;
};

// Complete immutable mesh state held by `NavMesh::create`.
struct MeshImpl {
    std::vector<TriCell> cells;
};

// Validates the raw triangle list per MSH-004 and, on success, returns the
// complete mesh state including shared-edge adjacency (MSH-005). Transactional:
// any error leaves no partial state.
[[nodiscard]] Result<MeshImpl> build_mesh_impl(const std::vector<Polygon>& triangles);

// MSH-006 for one cell: strict interior, or within `epsilon` of any of the
// three closed edge segments.
[[nodiscard]] bool cell_contains(const TriCell& cell, Vec2 point) noexcept;

// MSH-006 over all cells: non-finite point -> false; strict interior -> true;
// within `epsilon` of any triangle edge segment -> true; otherwise false.
// Const and allocation-free.
[[nodiscard]] bool mesh_contains(const MeshImpl& impl, Vec2 point);

} // namespace vwmini::detail

// Completes the private PIMPL type declared in the public header (a member
// definition is access-unrestricted). It lives here, in the namespace that
// encloses `NavMesh`, so every translation unit with friend access can use it.
namespace vwmini {
struct NavMesh::Impl {
    detail::MeshImpl data;
};
} // namespace vwmini
