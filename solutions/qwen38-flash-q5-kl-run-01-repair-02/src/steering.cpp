#include "steering.hpp"

#include "predicates.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <span>

namespace vwmini::detail
{
namespace
{

/// How far ahead (seconds) a pair of discs is tested for a meeting.
inline constexpr double kLookAhead = 0.75;

/// Fraction of the faster disc's speed below which a pair counts as "not moving" in the
/// meeting test. Relative, so the rule reads the same at 1 mm/s and at 1e6 m/s.
constexpr double kStillSpeedFraction = 1.0e-6;

/// Fraction of the combined radii below which two centres count as concentric, where "apart"
/// has no direction. Relative for the same reason.
constexpr double kConcentricFraction = 1.0e-6;

/// Narrowest float that still keeps a computed velocity representable.
[[nodiscard]] float to_float_clamped(double value) noexcept
{
    constexpr double kLimit = static_cast<double>(std::numeric_limits<float>::max());
    return static_cast<float>(std::clamp(value, -kLimit, kLimit));
}

/**
 * Policy double -> float for a velocity that has already been limited by `max_speed`.
 *
 * Narrowing rounds, and rounding can put the stored magnitude a float ULP over the limit. The
 * limit is a contract cap, not an approximate target, so the value is pulled back down.
 */
[[nodiscard]] Vec2 to_velocity(Vec2d value, float max_speed) noexcept
{
    Vec2 narrowed{to_float_clamped(value.x), to_float_clamped(value.y)};
    if (max_speed <= 0.0f)
    {
        return narrowed;
    }
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        const double speed = magnitude(Vec2d(narrowed));
        if (!(speed > static_cast<double>(max_speed)))
        {
            break;
        }
        narrowed = narrowed * (max_speed / static_cast<float>(speed));
    }
    return narrowed;
}

[[nodiscard]] double wrap_angle(double angle) noexcept
{
    constexpr double kPi = std::numbers::pi;
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
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

/// Keep the speed at or below `max_speed`, computed in the policy's double domain.
[[nodiscard]] Vec2d clamp_speed(Vec2d velocity, float max_speed) noexcept
{
    const double limit = static_cast<double>(max_speed);
    if (limit <= 0.0)
    {
        return velocity;
    }
    const double speed = magnitude(velocity);
    if (speed <= limit)
    {
        return velocity;
    }
    return velocity * (limit / speed);
}

/// Deterministic separation axis for two discs sitting on the very same point.
[[nodiscard]] Vec2d concentric_axis(std::size_t index, std::size_t peer) noexcept
{
    return (index < peer) ? Vec2d{1.0, 0.0} : Vec2d{-1.0, 0.0};
}

/**
 * True when `relative` motion brings the two discs together inside the look-ahead horizon.
 *
 * `to_peer` points from this agent to the peer, `relative` is the peer's motion in this
 * agent's frame, so the peer sweeps the segment `to_peer -> to_peer + relative * horizon`.
 * The pair meets exactly when that segment passes within the combined radii of this agent at
 * the origin; separating pairs fail because their closest point is the start of the segment.
 */
[[nodiscard]] bool meets_within_horizon(Vec2d to_peer, Vec2d relative, double combined_radius,
                                        double still_speed) noexcept
{
    if (dot_d(relative, relative) <= still_speed * still_speed)
    {
        return false;
    }
    const double sweep = combined_radius * combined_radius;
    return distance_squared_to_segment(Vec2d{}, to_peer, to_peer + relative * kLookAhead) <= sweep;
}

/**
 * Rotate `relative` onto the near edge of the pair's collision cone, keeping its speed.
 *
 * The cone's half angle is `asin(combined_radius / gap)`, valid because the caller has
 * excluded `gap <= combined_radius`.
 */
[[nodiscard]] Vec2d deflect_onto_cone_edge(Vec2d to_peer, Vec2d relative, double combined_radius,
                                           double gap) noexcept
{
    const double half_angle = std::asin(std::min(1.0, combined_radius / gap));
    const double collision_bearing = std::atan2(-to_peer.y, -to_peer.x);
    const double heading = std::atan2(relative.y, relative.x);
    // Exactly head-on pairs take the clockwise side: fixed, so the choice is deterministic.
    const double side = (wrap_angle(heading - collision_bearing) > 0.0) ? 1.0 : -1.0;
    const double deflected_heading = collision_bearing + side * half_angle;
    const double speed = magnitude(relative);
    return Vec2d{std::cos(deflected_heading), std::sin(deflected_heading)} * speed;
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
    // The offset is subtracted in double: two accepted coordinates may sit further apart than
    // `FLT_MAX`, and float arithmetic there would overflow before normalization could help.
    // `unit` then divides the scale out instead of overflowing on the way.
    const Vec2d offset = Vec2d(*target) - Vec2d(agent.position);
    return to_velocity(clamp_speed(unit(offset) * double(agent.max_speed), agent.max_speed),
                       agent.max_speed);
}

Vec2 avoid_velocity(std::span<const AgentRuntime> snapshot, std::size_t index, Vec2 desired)
{
    const AgentRuntime &self = snapshot[index];
    Vec2d velocity = desired;

    for (std::size_t peer_index = 0; peer_index < snapshot.size(); ++peer_index)
    {
        if (peer_index == index)
        {
            continue;
        }
        const AgentRuntime &peer = snapshot[peer_index];
        const Vec2d to_peer = Vec2d(peer.position) - Vec2d(self.position);
        const double combined_radius = double(self.radius) + double(peer.radius);
        const double gap = magnitude(to_peer);

        if (gap <= combined_radius)
        {
            // Already overlapping: drive apart along the centre line (SIM-010 robustness).
            const Vec2d axis = (gap > combined_radius * kConcentricFraction)
                                   ? to_peer / gap
                                   : concentric_axis(index, peer_index);
            velocity = axis * -double(self.max_speed);
            continue;
        }

        // Motion of the peer in this agent's frame, relative to the velocity chosen so far.
        const Vec2d relative = Vec2d(peer.velocity) - velocity;
        const double still_speed =
            kStillSpeedFraction * std::max(double(self.max_speed), double(peer.max_speed));
        if (!meets_within_horizon(to_peer, relative, combined_radius, still_speed))
        {
            continue;
        }
        velocity =
            Vec2d(peer.velocity) - deflect_onto_cone_edge(to_peer, relative, combined_radius, gap);
    }

    return to_velocity(clamp_speed(velocity, self.max_speed), self.max_speed);
}

} // namespace vwmini::detail
