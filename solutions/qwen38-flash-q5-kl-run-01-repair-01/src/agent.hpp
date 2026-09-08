#pragma once

#include <vwmini/simulation.hpp>

#include "predicates.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace vwmini::detail
{

/**
 * One agent's mutable state plus the transitions that depend only on that state.
 *
 * The route is the remainder of the route polyline still to walk; `goal` outlives it so
 * arrival can be judged against the caller's goal point rather than a waypoint.
 */
struct AgentRuntime
{
    std::uint32_t id{};
    Vec2 position{};
    Vec2 velocity{};
    float radius{};
    float max_speed{};
    std::optional<Vec2> goal{};
    float arrival_radius{};
    std::vector<Vec2> route;
    /**
     * Travel a substep intended but the stored float position could not represent.

     * A position is a `Vec2`, so a step shorter than one float ULP of the current coordinate
     * rounds away. Far from the origin that is a real distance: at 2e38 m one ULP is about 4e31 m.
     * Keeping the remainder here and offering it again next substep keeps motion additive instead
     * of silently freezing an agent that is doing nothing wrong.
     */
    Vec2d residual{};
    AgentStatus status{AgentStatus::Idle};

    [[nodiscard]] AgentState state() const noexcept
    {
        return AgentState{position, velocity, radius, max_speed, goal, status};
    }

    /// SIM-007 transitions for a freshly computed route: empty means "no route exists".
    void adopt_route(std::vector<Vec2> new_route) noexcept
    {
        if (new_route.empty())
        {
            status = AgentStatus::NoPath;
            velocity = Vec2{};
            return;
        }
        if (goal.has_value() && distance(*goal, position) <= double(arrival_radius))
        {
            route.clear();
            velocity = Vec2{};
            status = AgentStatus::Reached;
            return;
        }
        // The route starts at the agent's own position; keeping that point would waste the
        // first substep, because motion toward it is zero by definition.
        while (!new_route.empty() && distance(new_route.front(), position) <= double(kEpsilon))
        {
            new_route.erase(new_route.begin());
        }
        status = AgentStatus::Moving;
        route = std::move(new_route);
    }

    /// SIM-007: dropping the goal also drops the route, arrival state and velocity.
    void clear_goal() noexcept
    {
        goal = std::nullopt;
        route.clear();
        velocity = Vec2{};
        status = AgentStatus::Idle;
    }

    /// The point the agent is currently heading for: the next waypoint, else the goal.
    [[nodiscard]] const Vec2 *target() const noexcept
    {
        if (!route.empty())
        {
            return &route.front();
        }
        return goal.has_value() ? &goal.value() : nullptr;
    }

    /// SIM-009: arrival is measured against the goal, not a waypoint.
    void refresh_arrival() noexcept
    {
        if (status != AgentStatus::Moving || !goal.has_value())
        {
            return;
        }
        if (distance(position, *goal) <= double(arrival_radius))
        {
            route.clear();
            velocity = Vec2{};
            status = AgentStatus::Reached;
        }
    }
};

} // namespace vwmini::detail
