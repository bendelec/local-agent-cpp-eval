#pragma once

#include "mesh_topology.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace vwmini::detail
{

/**
 * Cheapest chain of adjacent cells joining any cell of `starts` to any cell of `goals`.
 *
 * Dijkstra with the Euclidean centroid-to-centroid distance as step cost. Selection always
 * takes the cheapest unsettled cell and, on equal cost, the lowest cell index, so the
 * returned corridor is deterministic for a given mesh (SIM-004). Returns an empty vector
 * when the two cell sets lie in different components.
 */
[[nodiscard]] std::vector<std::size_t> find_corridor(const MeshTopology &mesh,
                                                     std::span<const std::size_t> starts,
                                                     std::span<const std::size_t> goals);

} // namespace vwmini::detail
