#include <vwmini/simulation.hpp>

#include "agent.hpp"
#include "error.hpp"
#include "nav_mesh_impl.hpp"
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

/// How many times an out-of-mesh displacement is bisected before the agent takes what fits.
///
/// The bound caps the exact coverage queries of one substep. Half of 2^-16 of a stride is below
/// the float ULP of any coordinate the stride itself starts from, so the shortening left over is
/// not a distance an agent could have travelled anyway.
constexpr int kMotionBisectionSteps = 16;

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
 * The largest fraction of `desired` whose stored position segment stays inside the mesh.
 *
 * The question is asked of `MeshTopology::segment_is_contained` — the same exact coverage
 * primitive that certifies a planned route — so an accepted substep is as trustworthy as a path:
 * every point of the segment between the old and the stored position is walkable (SIM-012).
 * Coverage along a ray is not monotone in general, so bisection returns a verified fraction
 * rather than necessarily the largest one; verifying the candidate, not maximising it, is what
 * the contract requires. The whole step is tried first, so unobstructed motion costs one query.
 */
[[nodiscard]] double contained_fraction(const detail::MeshTopology &topology, Vec2 origin,
                                        Vec2d desired)
{
    const auto whole = (Vec2d{origin} + desired).narrow();
    if (whole && topology.segment_is_contained(origin, *whole))
    {
        return 1.0;
    }
    double fits = 0.0;
    double too_far = 1.0;
    for (int attempt = 0; attempt < kMotionBisectionSteps; ++attempt)
    {
        const double middle = 0.5 * (fits + too_far);
        // Each candidate is judged as the float position it would become, because that is the
        // position that would actually be stored and used by every later query.
        const auto landing = (Vec2d{origin} + desired * middle).narrow();
        if (landing && topology.segment_is_contained(origin, *landing))
        {
            fits = middle;
        }
        else
        {
            too_far = middle;
        }
    }
    return fits;
}

/**
 * The displacement an agent may actually take: `desired`, shortened until the mesh covers the
 * segment to the stored position (SIM-008/SIM-012).
 *
 * Shortening rather than teleporting keeps motion continuous; if nothing fits, the agent stays
 * where it is for this substep.
 */
[[nodiscard]] Vec2d contained_displacement(const detail::MeshTopology &topology, Vec2d from,
                                           Vec2d desired)
{
    // `from` is a stored position, hence a `float`, so the narrowing cannot fail; checking it is
    // what keeps the conversion free of implementation-defined behaviour.
    const auto origin = from.narrow();
    if (!origin)
    {
        return Vec2d{};
    }
    return desired * contained_fraction(topology, *origin, desired);
}

/**
 * Move one agent by at most one substep and update its route progress.
 *
 * Agents that are not `Moving` are brought to rest; a substep may stop an agent exactly on
 * its target but never carries it past that target (SIM-008).
 */
void advance_agent(const detail::MeshTopology &topology, AgentRuntime &agent, float seconds)
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
        // (SIM-008).
        const double stride = std::min(speed * double(seconds), distance(previous, Vec2d(*target)));
        // Nothing is remembered between substeps: a displacement the stored `float` position
        // cannot represent simply does not happen, so the position never moves further than the
        // elapsed time allows (see architecture.md "Motion").
        const Vec2d applied =
            contained_displacement(topology, previous, Vec2d(agent.velocity) * (stride / speed));
        // `contained_displacement` only ever shortens toward a walkable endpoint; what is left
        // still has to be a representable position, and `narrow` checks the range before it
        // converts. A candidate that fails is dropped, leaving the agent finite and in place.
        const auto landed = (previous + applied).narrow();
        if (landed)
        {
            agent.position = *landed;
        }
    }

    // Drop waypoints the agent actually reached: on the travelled segment and no longer: on the
    // travelled segment and no longer ahead of it. A waypoint merely passed to the side stays, so
    // avoidance cannot erase the corner of a route and leave the agent steering straight through a
    // wall.
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

    /// Motion's exact segment query, taken from the mesh's own validated topology. This is the
    /// only place `Simulation` reaches into `NavMesh`: `NavMesh` grants it friendship, and no
    /// public declaration changes.
    [[nodiscard]] const detail::MeshTopology &topology() const noexcept
    {
        return mesh.m_impl->topology;
    }

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
            advance_agent(m_impl->topology(), m_impl->agents[index], substep);
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
