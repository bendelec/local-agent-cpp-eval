#include "steering.hpp"

#include "predicates.hpp"

#include <cmath>
#include <numbers>
#include <span>

namespace vwmini::detail
{
namespace
{

/// How far ahead (seconds) a pair of discs is tested for a meeting.
inline constexpr float kLookAhead = 0.75f;

/// Relative speeds below this are treated as "not moving" for the meeting test.
constexpr float kStillRelativeSpeed = 1.0e-6f;

/// Centre separations below this count as concentric, where "apart" has no direction.
constexpr float kConcentricGap = 1.0e-6f;

[[nodiscard]] float wrap_angle(float angle) noexcept
{
    constexpr float kPi = std::numbers::pi_v<float>;
    constexpr float kTwoPi = 2.0f * kPi;
    while (angle > kPi)
    {
        angle -= kTwoPi;
    }
    while (angle < -kPi)
    {
        angle += kTwoPi;
    }
    return angle;
}

[[nodiscard]] Vec2 clamp_speed(Vec2 velocity, float max_speed) noexcept
{
    const float speed = length(velocity);
    if (max_speed <= 0.0f || speed <= max_speed)
    {
        return velocity;
    }
    return velocity * (max_speed / speed);
}

/// Deterministic separation axis for two discs sitting on the very same point.
[[nodiscard]] Vec2 concentric_axis(std::size_t index, std::size_t peer) noexcept
{
    return (index < peer) ? Vec2{1.0f, 0.0f} : Vec2{-1.0f, 0.0f};
}

/**
 * True when `relative` motion brings the two discs together inside the look-ahead horizon.
 *
 * `to_peer` points from this agent to the peer, `relative` is the peer's motion in this
 * agent's frame, so the peer sweeps the segment `[to_peer, to_peer + relative * horizon]`.
 * The pair meets exactly when that segment passes within the combined radii of this agent
 * at the origin; separating pairs fail the test because their closest point is the start of
 * the segment, which is already further away than the combined radii.
 */
[[nodiscard]] bool meets_within_horizon(Vec2 to_peer, Vec2 relative, float combined_radius) noexcept
{
    if (dot(relative, relative) <= kStillRelativeSpeed * kStillRelativeSpeed)
    {
        return false;
    }
    const float sweep = combined_radius * combined_radius;
    return distance_squared_to_segment(Vec2{}, to_peer, to_peer + relative * kLookAhead) <= sweep;
}

/**
 * Rotate `relative` onto the nearer edge of the pair's collision cone, keeping its speed.
 *
 * The cone's axis is the direction in which the peer would reach this agent; its half angle
 * is `asin(combined_radius / gap)`, valid because the caller has excluded `gap <= radius`.
 */
[[nodiscard]] Vec2 deflect_onto_cone_edge(Vec2 to_peer, Vec2 relative, float combined_radius,
                                          float gap) noexcept
{
    const float half_angle = std::asin(combined_radius / gap);
    const float collision_bearing = std::atan2(-to_peer.y, -to_peer.x);
    const float heading = std::atan2(relative.y, relative.x);
    // Exactly head-on pairs take the clockwise side: fixed, so the choice is deterministic.
    const float side = (wrap_angle(heading - collision_bearing) > 0.0f) ? 1.0f : -1.0f;
    const float deflected_heading = collision_bearing + side * half_angle;
    const float speed = length(relative);
    return Vec2{std::cos(deflected_heading), std::sin(deflected_heading)} * speed;
}

} // namespace

Vec2 seek_velocity(const AgentRuntime &agent) noexcept
{
    if (agent.status != AgentStatus::Moving)
    {
        return Vec2{};
    }
    const Vec2 *target = agent.target();
    if (target == nullptr)
    {
        return Vec2{};
    }
    return normalized(*target - agent.position) * agent.max_speed;
}

Vec2 avoid_velocity(std::span<const AgentRuntime> snapshot, std::size_t index, Vec2 desired)
{
    const AgentRuntime &self = snapshot[index];
    Vec2 velocity = desired;

    for (std::size_t peer_index = 0; peer_index < snapshot.size(); ++peer_index)
    {
        if (peer_index == index)
        {
            continue;
        }
        const AgentRuntime &peer = snapshot[peer_index];
        const Vec2 to_peer = peer.position - self.position;
        const float combined_radius = self.radius + peer.radius;
        const float gap = length(to_peer);

        if (gap <= combined_radius)
        {
            // Already overlapping: drive apart along the centre line (SIM-010 robustness).
            const Vec2 axis = (gap > kConcentricGap) ? to_peer * (1.0f / gap)
                                                     : concentric_axis(index, peer_index);
            velocity = axis * -self.max_speed;
            continue;
        }

        // Motion of the peer in this agent's frame, relative to the velocity chosen so far.
        const Vec2 relative = peer.velocity - velocity;
        if (!meets_within_horizon(to_peer, relative, combined_radius))
        {
            continue;
        }
        velocity = peer.velocity - deflect_onto_cone_edge(to_peer, relative, combined_radius, gap);
    }

    return clamp_speed(velocity, self.max_speed);
}

} // namespace vwmini::detail
