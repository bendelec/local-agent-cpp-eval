// Internal PIMPL state and helpers for the public `vwmini::Simulation`
// (include/vwmini/simulation.hpp). Completes `Simulation::Impl`.
//
// This header assumes the public header has already been included (it
// provides `Simulation`, `AgentStatus`, `AgentId`, etc.).
#pragma once

#include <vwmini/simulation.hpp>
#include <vwmini/nav_mesh.hpp>

#include "vwmini/geometry_priv.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace vwmini {

// One agent slot; slot index `i` corresponds to `AgentId{ i + 1 }`.
struct SimAgent {
    bool alive{false};
    Vec2 position{};
    Vec2 velocity{};
    float radius{};
    float max_speed{};
    std::optional<Vec2> goal{};
    float arrival_radius{};  ///< Effective value (>= 0), resolved at goal time.
    AgentStatus status{AgentStatus::Idle};
    std::vector<Vec2> route;  ///< Waypoints; excludes the start point.
    std::size_t route_index{0};  ///< Index of the current target waypoint.
};

// Completes the private PIMPL type declared in the public header (a member
// definition is access-unrestricted). It lives in the enclosing namespace so
// every translation unit that instantiates `Simulation` members can see it.
struct Simulation::Impl {
    NavMesh mesh;
    std::vector<SimAgent> agents;  ///< Slot i == AgentId{ i + 1 }.
    std::uint32_t next_id{1};  ///< Monotonic; never reused after removal.
    std::size_t live_count{0};

    explicit Impl(NavMesh mesh_in) : mesh(std::move(mesh_in)) {}
};

// Applies the shared goal state transition (Moving / Reached / NoPath) to an
// agent. Always succeeds once the preconditions (finite, in-mesh) are met.
void apply_goal(SimAgent& agent, Vec2 goal, float arrival_radius,
                const NavMesh& mesh);

} // namespace vwmini
