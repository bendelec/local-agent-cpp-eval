// Path finding (SIM-001..SIM-004): endpoint validation, direct segments,
// Dijkstra over cell adjacency, funnel string-pulling over shared-edge
// portals, and exact segment-containment verification.
#pragma once

#include <vwmini/nav_mesh.hpp>

#include "vwmini/mesh_priv.hpp"

namespace vwmini::detail {

// Exact MSH-006-based test: every real point of the closed segment [a, b]
// (including the epsilon tolerance band) is contained. No allocations on the
// success path are required; used for direct-path checks and output gates.
[[nodiscard]] bool segment_contained(const MeshImpl& mesh, Vec2 a, Vec2 b);

} // namespace vwmini::detail
