#include <vwmini/simulation.hpp>

#include "agent.hpp"
#include "error.hpp"
#include "predicates.hpp"
#include "steering.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace vwmini
{
namespace
{

using detail::AgentRuntime;
using detail::all_finite;
using detail::distance;
using detail::distance_squared_to_segment;
using detail::dot_d;
using detail::is_finite;
using detail::kEpsilonSquared;
using detail::magnitude;
using detail::Vec2d;

/// SIM-006's documented sentinel: "use the agent's own radius".
constexpr float kArrivalRadiusSentinel = -1.0f;

/// Preferred substep duration; `step` divides a duration into slices of about this size.
constexpr float kPreferredSubstep = 1.0e-2f;

/// Ceiling on substeps per `step` call: it bounds the work of one call, and for very large
/// durations it overrides `kPreferredSubstep` (the slices grow instead of the count).
constexpr int kMaxSubsteps = 10'000;

/// A displacement is shrunk by this factor until its endpoint is back inside the mesh.
constexpr double kContainmentShrink = 0.5;

/// How many times a displacement may be shrunk before the agent simply does not move.
constexpr int kContainmentAttempts = 12;

/// SIM-006: `-1.0f` is the only negative value that is not an error.
[[nodiscard]] bool arrival_radius_is_valid(float arrival_radius) noexcept
{
    return arrival_radius >= 0.0f || arrival_radius == kArrivalRadiusSentinel;
}

/// SIM-006: the sentinel means "borrow the agent's own radius".
[[nodiscard]] float resolve_arrival_radius(float requested, float fallback) noexcept
{
    return (requested == kArrivalRadiusSentinel) ? fallback : requested;
}

/// Validation shared by `add_agent` and `set_goal` for a goal point and arrival radius.
[[nodiscard]] Result<void> validate_goal(const NavMesh &mesh, Vec2 goal, float arrival_radius)
{
    if (!is_finite(goal) || !is_finite(arrival_radius))
    {
        return std::unexpected(detail::make_error(ErrorCode::InvalidArgument,
                                                  "goal and arrival radius must be finite"));
    }
    if (!arrival_radius_is_valid(arrival_radius))
    {
        return std::unexpected(detail::make_error(ErrorCode::InvalidArgument,
                                                  "arrival radius must be -1 or non-negative"));
    }
    if (!mesh.contains(goal))
    {
        return std::unexpected(
            detail::make_error(ErrorCode::OutsideMesh, "goal is outside the mesh"));
    }
    return {};
}

/**
 * Shorten `desired` until its endpoint stays inside the mesh (SIM-008/SIM-012).
 *
 * Shortening rather than teleporting keeps motion continuous; if nothing fits the agent
 * stays where it is for this substep.
 */
/**
 * True when both ends and the middle of `from` -> `to` lie in the mesh.
 *
 * Deliberately expressed through `NavMesh`'s public point query: motion stays independent of the
 * mesh's internal representation. The midpoint is the cheapest evidence that the way between the
 * two positions was not open ground all along, and shrinking the candidate on failure removes
 * most motion that would cross an obstacle in one substep.
 */
[[nodiscard]] bool sweep_is_inside(const NavMesh &mesh, Vec2 from, Vec2 to)
{
    if (!mesh.contains(from) || !mesh.contains(to))
    {
        return false;
    }
    // The mean of two finite floats cannot overflow in double, so the midpoint is representable.
    const Vec2 middle = ((Vec2d(from) + Vec2d(to)) * 0.5).to_vec2();
    if (!is_finite(middle))
    {
        return false;
    }
    return mesh.contains(middle);
}

[[nodiscard]] Vec2d contained_displacement(const NavMesh &mesh, Vec2d from, Vec2d desired)
{
    const Vec2 origin = from.to_vec2();
    Vec2d candidate = desired;
    for (int attempt = 0; attempt < kContainmentAttempts; ++attempt)
    {
        // The candidate is tested as the float position it would become, which is the position
        // that would actually be stored and used by every later query.
        const Vec2 landed = (from + candidate).to_vec2();
        // The swept sub-segment is checked as well as the landing point: every candidate lies
        // on one ray, so an endpoint-only test could cross an obstacle and land inside again
        // (SIM-012: no movement outside the mesh). `NavMesh` exposes point containment only, so
        // the sweep is decided by testing the midpoint of the swept segment too; see
        // architecture.md "Motion" for what this does and does not guarantee.
        if (is_finite(landed) && sweep_is_inside(mesh, origin, landed))
        {
            return candidate;
        }
        candidate = candidate * kContainmentShrink;
    }
    return Vec2d{};
}

/**
 * Move one agent by at most one substep and update its route progress.
 *
 * Agents that are not `Moving` are brought to rest; a substep may stop an agent exactly on
 * its target but never carries it past that target (SIM-008).
 */
void advance_agent(const NavMesh &mesh, AgentRuntime &agent, float seconds)
{
    if (agent.status != AgentStatus::Moving)
    {
        agent.velocity = Vec2{};
        return;
    }
    const Vec2 *target = agent.target();
    if (target == nullptr)
    {
        agent.velocity = Vec2{};
        return;
    }

    const Vec2d previous = agent.position;
    const double speed = magnitude(agent.velocity);
    if (speed > 0.0)
    {
        // A substep may stop the agent exactly on its target but never carry it past one
        // (SIM-008), and the leftover from earlier substeps counts towards that limit.
        const double remaining = distance(previous, Vec2d(*target));
        Vec2d wanted = agent.residual +
                       Vec2d(agent.velocity) * std::min(speed * double(seconds), remaining) / speed;
        const double wanted_length = magnitude(wanted);
        if (wanted_length > remaining && wanted_length > 0.0)
        {
            wanted = wanted * (remaining / wanted_length);
        }
        const Vec2d applied = contained_displacement(mesh, previous, wanted);
        const Vec2d intended = previous + applied;
        // Motion that would leave the float range is not applied: the position stays finite and
        // the agent simply cannot travel that far in one substep. What the float position cannot
        // hold — a step below one ULP of the coordinate — is not lost either: it goes to
        // `residual` and is tried again next substep (see architecture.md "Motion").
        const Vec2 landed = intended.to_vec2();
        if (is_finite(landed))
        {
            agent.position = landed;
            agent.residual = intended - Vec2d(landed);
        }
    }

    // Drop waypoints the agent actually reached: on the travelled segment and no longer
    // ahead of it. A waypoint merely passed to the side stays, so avoidance cannot erase
    // the corner of a route and leave the agent steering straight through a wall.
    const Vec2d motion = agent.position - previous;
    while (!agent.route.empty())
    {
        const Vec2d waypoint = agent.route.front();
        const bool on_path =
            distance_squared_to_segment(waypoint, previous, agent.position) <= kEpsilonSquared;
        const bool behind = dot_d(waypoint - agent.position, motion) <= 0.0;
        if (!on_path || !behind)
        {
            break;
        }
        agent.route.erase(agent.route.begin());
    }
    agent.refresh_arrival();
}

} // namespace

struct Simulation::Impl
{
    explicit Impl(NavMesh value) : mesh(std::move(value)) {}

    [[nodiscard]] AgentRuntime *find_agent(AgentId id) noexcept
    {
        const auto found = std::ranges::find(agents, id.value, &AgentRuntime::id);
        return found == agents.end() ? nullptr : &*found;
    }

    [[nodiscard]] const AgentRuntime *find_agent(AgentId id) const noexcept
    {
        const auto found = std::ranges::find(agents, id.value, &AgentRuntime::id);
        return found == agents.end() ? nullptr : &*found;
    }

    /** SIM-007: re-route an agent whose goal has just been set. */
    void route_to_goal(AgentRuntime &agent)
    {
        if (!agent.goal.has_value())
        {
            agent.clear_goal();
            return;
        }
        auto route = find_path(mesh, agent.position, *agent.goal);
        // A `NoPath` goal is not an error: the agent reports the missing connection.
        agent.adopt_route(route.has_value() ? std::move(route->points) : std::vector<Vec2>{});
    }

    NavMesh mesh;
    std::vector<AgentRuntime> agents;
    std::uint32_t next_id{1};
};

Simulation::Simulation(NavMesh mesh) : m_impl(std::make_unique<Impl>(std::move(mesh))) {}

Simulation::~Simulation() = default;
Simulation::Simulation(Simulation &&) noexcept = default;
Simulation &Simulation::operator=(Simulation &&) noexcept = default;

Result<AgentId> Simulation::add_agent(const AgentConfig &config)
{
    if (!is_finite(config.position) || !is_finite(config.radius) || !is_finite(config.max_speed) ||
        !is_finite(config.arrival_radius) || (config.goal.has_value() && !is_finite(*config.goal)))
    {
        return std::unexpected(
            detail::make_error(ErrorCode::InvalidArgument, "agent configuration must be finite"));
    }
    if (config.radius <= 0.0f || config.max_speed <= 0.0f)
    {
        return std::unexpected(detail::make_error(ErrorCode::InvalidArgument,
                                                  "radius and maximum speed must be positive"));
    }
    if (!arrival_radius_is_valid(config.arrival_radius))
    {
        return std::unexpected(detail::make_error(ErrorCode::InvalidArgument,
                                                  "arrival radius must be -1 or non-negative"));
    }
    if (!m_impl->mesh.contains(config.position))
    {
        return std::unexpected(
            detail::make_error(ErrorCode::OutsideMesh, "agent position is outside the mesh"));
    }
    if (config.goal.has_value())
    {
        if (auto goal_check = validate_goal(m_impl->mesh, *config.goal, config.arrival_radius);
            !goal_check.has_value())
        {
            return std::unexpected(std::move(goal_check.error()));
        }
    }

    AgentRuntime agent;
    agent.id = m_impl->next_id++;
    agent.position = config.position;
    agent.residual = Vec2d{};
    agent.radius = config.radius;
    agent.max_speed = config.max_speed;
    agent.goal = config.goal;
    agent.arrival_radius = resolve_arrival_radius(config.arrival_radius, config.radius);
    m_impl->route_to_goal(agent);
    m_impl->agents.push_back(agent);
    return AgentId{agent.id};
}

Result<void> Simulation::remove_agent(AgentId id)
{
    AgentRuntime *agent = m_impl->find_agent(id);
    if (agent == nullptr)
    {
        return std::unexpected(detail::make_error(ErrorCode::NotFound, "unknown agent id"));
    }
    m_impl->agents.erase(m_impl->agents.begin() + (agent - m_impl->agents.data()));
    return {};
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius)
{
    AgentRuntime *agent = m_impl->find_agent(id);
    if (agent == nullptr)
    {
        return std::unexpected(detail::make_error(ErrorCode::NotFound, "unknown agent id"));
    }
    if (auto goal_check = validate_goal(m_impl->mesh, goal, arrival_radius);
        !goal_check.has_value())
    {
        return std::unexpected(std::move(goal_check.error()));
    }

    agent->goal = goal;
    agent->arrival_radius = resolve_arrival_radius(arrival_radius, agent->radius);
    m_impl->route_to_goal(*agent);
    return {};
}

Result<void> Simulation::clear_goal(AgentId id)
{
    AgentRuntime *agent = m_impl->find_agent(id);
    if (agent == nullptr)
    {
        return std::unexpected(detail::make_error(ErrorCode::NotFound, "unknown agent id"));
    }
    agent->clear_goal();
    return {};
}

Result<void> Simulation::step(float seconds)
{
    if (!is_finite(seconds) || seconds < 0.0f)
    {
        return std::unexpected(detail::make_error(ErrorCode::InvalidArgument,
                                                  "step duration must be finite and non-negative"));
    }
    if (seconds == 0.0f || m_impl->agents.empty())
    {
        return {};
    }

    // Counted in double: `seconds / kPreferredSubstep` alone overflows a float for a duration
    // near `FLT_MAX`, which would leave the clamp to `kMaxSubsteps` doing IEEE-dependent work.
    const double requested =
        std::min(double(seconds) / static_cast<double>(kPreferredSubstep) + 1.0,
                 static_cast<double>(kMaxSubsteps));
    const int substeps = static_cast<int>(requested);
    const float substep = seconds / static_cast<float>(substeps);

    // Reused across substeps: decisions read a snapshot taken before any agent moved.
    std::vector<AgentRuntime> snapshot;
    for (int iteration = 0; iteration < substeps; ++iteration)
    {
        snapshot = m_impl->agents; // SIM-010: every agent decides from the same state.
        for (std::size_t index = 0; index < snapshot.size(); ++index)
        {
            const Vec2 desired = (snapshot[index].status == AgentStatus::Moving)
                                     ? detail::seek_velocity(snapshot[index])
                                     : Vec2{};
            m_impl->agents[index].velocity = detail::avoid_velocity(snapshot, index, desired);
            advance_agent(m_impl->mesh, m_impl->agents[index], substep);
        }
    }
    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept
{
    const AgentRuntime *agent = m_impl->find_agent(id);
    return (agent == nullptr) ? std::nullopt : std::optional<AgentState>{agent->state()};
}

std::size_t Simulation::agent_count() const noexcept
{
    return m_impl->agents.size();
}

} // namespace vwmini
