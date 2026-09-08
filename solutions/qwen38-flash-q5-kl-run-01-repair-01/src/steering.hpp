#pragma once

#include "agent.hpp"

#include <cstddef>
#include <span>

namespace vwmini::detail
{

/**
 * Unobstructed velocity of `agent` toward whatever it is heading for (SIM-008).
 *
 * Returns zero unless the agent is `Moving` (idle, goal-less, reached or unrouted agents do
 * not steer), so callers never inspect `AgentStatus` themselves.
 */
[[nodiscard]] Vec2 seek_velocity(const AgentRuntime &agent) noexcept;

/**
 * Choose the velocity one agent applies this substep (SIM-010/SIM-012).
 *
 * `desired` is the unobstructed velocity toward the agent's own next waypoint. Each peer
 * that the agent would meet inside the look-ahead horizon deflects the candidate velocity
 * onto the nearest boundary of that pair's collision cone, so the deflection is the
 * smallest rotation that stops the discs from meeting — no tuned repulsion weights. Peers
 * already overlapping are driven apart along the centre line instead. Peers that are
 * separating, or that only come close beyond the horizon, are ignored, so a follower does
 * not slow a leader that is moving away from it.
 *
 * Peers are screened in snapshot order and each deflection refines the velocity chosen so
 * far, which makes the choice deterministic but deliberately order-dependent.
 *
 * Reads the snapshot only, never writes it, so every agent can decide from the same
 * snapshot. The result is finite and never faster than the agent's own maximum speed.
 */
[[nodiscard]] Vec2 avoid_velocity(std::span<const AgentRuntime> snapshot, std::size_t index,
                                  Vec2 desired);

} // namespace vwmini::detail
