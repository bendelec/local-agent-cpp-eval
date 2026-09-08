#include <vwmini/nav_mesh.hpp>

#include "corridor.hpp"
#include "error.hpp"
#include "mesh_topology.hpp"
#include "nav_mesh_impl.hpp"
#include "predicates.hpp"

#include <cstddef>
#include <utility>
#include <vector>

namespace vwmini
{
namespace
{

using detail::kEpsilonSquared;
using detail::length_squared;
using detail::make_error;
using detail::Vec2d;

/**
 * String pulling over the corridor: repeatedly jump to the furthest waypoint that the
 * current point can see, where "see" means the segment stays inside the mesh. Every
 * retained segment is therefore verified, not merely assumed, to be walkable (SIM-002).
 */
[[nodiscard]] std::vector<Vec2> pull_string(const detail::MeshTopology &topology,
                                            const std::vector<Vec2> &route)
{
    std::vector<Vec2> taut;
    taut.push_back(route.front());
    std::size_t current = 0;
    while (current + 1 < route.size())
    {
        std::size_t next = current + 1;
        for (std::size_t candidate = route.size() - 1; candidate > current + 1; --candidate)
        {
            if (topology.segment_is_contained(route[current], route[candidate]))
            {
                next = candidate;
                break;
            }
        }
        taut.push_back(route[next]);
        current = next;
    }
    return taut;
}

/// Omit intermediate waypoints within epsilon of the previous kept point (SIM-002). The
/// caller's start and goal are never dropped.
[[nodiscard]] std::vector<Vec2> drop_near_waypoints(const std::vector<Vec2> &route)
{
    std::vector<Vec2> kept;
    kept.push_back(route.front());
    for (std::size_t index = 1; index + 1 < route.size(); ++index)
    {
        if (length_squared(Vec2d(route[index]) - Vec2d(kept.back())) > kEpsilonSquared)
        {
            kept.push_back(route[index]);
        }
    }
    kept.push_back(route.back());
    return kept;
}

[[nodiscard]] bool all_segments_contained(const detail::MeshTopology &topology,
                                          const std::vector<Vec2> &route)
{
    for (std::size_t index = 0; index + 1 < route.size(); ++index)
    {
        if (!topology.segment_is_contained(route[index], route[index + 1]))
        {
            return false;
        }
    }
    return true;
}

} // namespace

Result<Path> find_path(const NavMesh &mesh, Vec2 start, Vec2 goal)
{
    if (!detail::is_finite(start) || !detail::is_finite(goal))
    {
        return std::unexpected(
            make_error(ErrorCode::InvalidArgument, "path endpoints must be finite"));
    }
    const detail::MeshTopology &topology = mesh.m_impl->topology;
    if (!topology.contains(start))
    {
        return std::unexpected(
            make_error(ErrorCode::OutsideMesh, "path start is outside the mesh"));
    }
    if (!topology.contains(goal))
    {
        return std::unexpected(make_error(ErrorCode::OutsideMesh, "path goal is outside the mesh"));
    }
    if (start == goal)
    {
        return Path{{start}}; // SIM-001: exactly equal endpoints report a single point.
    }
    if (topology.segment_is_contained(start, goal))
    {
        return Path{{start, goal}};
    }

    const auto start_cells = topology.locate(start);
    const auto goal_cells = topology.locate(goal);
    const std::vector<std::size_t> corridor =
        detail::find_corridor(topology, start_cells, goal_cells);
    if (corridor.empty())
    {
        return std::unexpected(
            make_error(ErrorCode::NoPath, "no adjacency connects the endpoint cells"));
    }

    // Seed the string with both endpoints of every portal (the edge two corridor cells
    // share). Both endpoints of consecutive portals lie in their common cell, which is
    // convex, so the seeded polyline is contained by construction and the pull below can
    // only shorten it. Pulling against portal endpoints is what lets the route hug corners
    // instead of stalling at cell centres.
    std::vector<Vec2> route;
    route.push_back(start);
    for (std::size_t index = 0; index + 1 < corridor.size(); ++index)
    {
        const auto portal = topology.shared_edge(corridor[index], corridor[index + 1]);
        if (portal.has_value())
        {
            route.push_back(portal->first);
            route.push_back(portal->second);
        }
    }
    route.push_back(goal);

    std::vector<Vec2> taut = pull_string(topology, route);
    std::vector<Vec2> result = drop_near_waypoints(taut);
    if (!all_segments_contained(topology, result))
    {
        // Dropping merges two kept points only when the merged segment is contained, so a
        // failure means the pull itself produced a segment the topology rejects. Hand back the
        // shortest candidate that is verified: the pulled route, else the seeded corridor
        // polyline, which is contained by construction.
        if (!all_segments_contained(topology, taut))
        {
            taut = route;
        }
        result = std::move(taut);
    }
    return Path{std::move(result)};
}

} // namespace vwmini
