#pragma once

// Definition of the private representation of `NavMesh` (declared but not defined in the
// public header). Included only by the library's own translation units.

#include <vwmini/nav_mesh.hpp>

#include "mesh_topology.hpp"

#include <utility>

namespace vwmini
{

/// The whole of `NavMesh`'s state: one immutable, already-validated topology.
struct NavMesh::Impl
{
    detail::MeshTopology topology;
};

} // namespace vwmini
