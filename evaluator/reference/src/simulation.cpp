#include "geometry_internal.hpp"

#include <vwmini/simulation.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <ranges>
#include <string>
#include <utility>
#include <vector>

namespace vwmini {
namespace {

constexpr float kSubstepSeconds = 1.0f / 60.0f;
constexpr float kArrivalEpsilon = 1.0e-4f;
constexpr float kOverlapTolerance = 1.0e-3f;

[[nodiscard]] float distance(Vec2 a, Vec2 b) noexcept
{
    return length(a - b);
}

[[nodiscard]] Vec2 perpendicular_left(Vec2 value) noexcept
{
    return {-value.y, value.x};
}

[[nodiscard]] Vec2 capped(Vec2 velocity, float maximum) noexcept
{
    const float speed = length(velocity);
    return speed > maximum && speed > 0.0f ? velocity * (maximum / speed) : velocity;
}

[[nodiscard]] Error fail(ErrorCode code, std::string message)
{
    return Error{code, std::move(message)};
}

[[nodiscard]] Result<float> effective_arrival_radius(float requested, float radius)
{
    if (!std::isfinite(requested) || (requested < 0.0f && requested != -1.0f)) {
        return std::unexpected(fail(ErrorCode::InvalidArgument, "invalid arrival radius"));
    }
    return requested == -1.0f ? radius : requested;
}

struct AgentRecord {
    AgentId id{};
    AgentState state{};
    float arrival_radius{};
    Path route{};
    std::size_t waypoint{};
};

[[nodiscard]] Vec2 desired_velocity(const AgentRecord& agent, float seconds) noexcept
{
    if (agent.state.status != AgentStatus::Moving || agent.waypoint >= agent.route.points.size()) {
        return {};
    }
    const Vec2 direction = agent.route.points[agent.waypoint] - agent.state.position;
    const float remaining = length(direction);
    const float speed = std::min(agent.state.max_speed, remaining / seconds);
    return normalized(direction) * speed;
}

} // namespace

struct Simulation::Impl {
    explicit Impl(NavMesh value)
        : mesh(std::move(value))
    {
    }

    NavMesh mesh;
    std::vector<AgentRecord> agents;
    std::uint32_t next_id{1};

    [[nodiscard]] AgentRecord* find(AgentId id) noexcept
    {
        const auto found = std::ranges::find(agents, id, &AgentRecord::id);
        return found == agents.end() ? nullptr : &*found;
    }

    [[nodiscard]] const AgentRecord* find(AgentId id) const noexcept
    {
        const auto found = std::ranges::find(agents, id, &AgentRecord::id);
        return found == agents.end() ? nullptr : &*found;
    }

    [[nodiscard]] Result<void> assign_goal(AgentRecord& agent, Vec2 goal,
                                            float requested_arrival_radius)
    {
        if (!geom::is_finite(goal)) {
            return std::unexpected(fail(ErrorCode::InvalidArgument, "non-finite goal"));
        }
        if (!mesh.contains(goal)) {
            return std::unexpected(fail(ErrorCode::OutsideMesh, "goal outside mesh"));
        }
        const Result<float> arrival = effective_arrival_radius(requested_arrival_radius,
                                                                 agent.state.radius);
        if (!arrival) {
            return std::unexpected(arrival.error());
        }

        agent.state.goal = goal;
        agent.arrival_radius = *arrival;
        agent.route.points.clear();
        agent.waypoint = 0;
        agent.state.velocity = {};
        if (distance(agent.state.position, goal) <= agent.arrival_radius + kArrivalEpsilon) {
            agent.state.status = AgentStatus::Reached;
            return {};
        }

        const Result<Path> path = find_path(mesh, agent.state.position, goal);
        if (!path) {
            if (path.error().code == ErrorCode::NoPath) {
                agent.state.status = AgentStatus::NoPath;
                return {};
            }
            return std::unexpected(path.error());
        }
        agent.route = *path;
        agent.waypoint = agent.route.points.size() > 1u ? 1u : 0u;
        agent.state.status = AgentStatus::Moving;
        return {};
    }

    void advance_route(AgentRecord& agent) noexcept
    {
        if (agent.state.status != AgentStatus::Moving || !agent.state.goal) {
            return;
        }
        if (distance(agent.state.position, *agent.state.goal) <=
            agent.arrival_radius + kArrivalEpsilon) {
            // Arrival is radius-based; do not teleport through a neighbor that was
            // safely avoided during this substep.
            agent.state.velocity = {};
            agent.state.status = AgentStatus::Reached;
            agent.route.points.clear();
            agent.waypoint = 0;
            return;
        }
        while (agent.waypoint + 1u < agent.route.points.size() &&
               distance(agent.state.position, agent.route.points[agent.waypoint]) <=
                   kArrivalEpsilon) {
            ++agent.waypoint;
        }
    }

    [[nodiscard]] bool safe_velocity(std::size_t index, Vec2 candidate,
                                     const std::vector<Vec2>& positions,
                                     const std::vector<Vec2>& desired,
                                     float seconds) const noexcept
    {
        const AgentRecord& subject = agents[index];
        const Vec2 next_position = positions[index] + candidate * seconds;
        for (std::size_t other = 0; other < agents.size(); ++other) {
            if (other == index) {
                continue;
            }
            const float minimum = subject.state.radius + agents[other].state.radius -
                                  kOverlapTolerance;
            // Stable id priority breaks otherwise symmetric local deadlocks. A lower-id
            // agent plans against a yielding higher-id neighbor; a higher-id agent
            // plans against the lower-id neighbor's intended velocity. Both decisions
            // still read only the same pre-substep snapshot.
            const Vec2 predicted_other = positions[other] +
                                         (agents[other].id.value < subject.id.value
                                              ? desired[other] * seconds
                                              : Vec2{});
            if (distance(next_position, predicted_other) < minimum) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] Vec2 overlap_escape_velocity(std::size_t index,
                                                const std::vector<Vec2>& positions,
                                                float seconds) const noexcept
    {
        const AgentRecord& subject = agents[index];
        Vec2 escape{};
        for (std::size_t other = 0; other < agents.size(); ++other) {
            if (other == index) {
                continue;
            }
            const Vec2 delta = positions[index] - positions[other];
            const float distance_to_other = length(delta);
            const float required = subject.state.radius + agents[other].state.radius;
            if (distance_to_other >= required) {
                continue;
            }
            // Coincident centres have no geometric escape normal. Stable id ordering
            // gives each agent an opposite deterministic direction instead of freezing.
            const Vec2 direction = distance_to_other > kArrivalEpsilon
                                       ? delta * (1.0f / distance_to_other)
                                       : (subject.id.value < agents[other].id.value
                                              ? Vec2{-1.0f, 0.0f}
                                              : Vec2{1.0f, 0.0f});
            escape = escape + direction * ((required - distance_to_other) / seconds);
        }
        return capped(escape, subject.state.max_speed);
    }

    [[nodiscard]] Vec2 contained_velocity(const AgentRecord& agent, Vec2 velocity,
                                          float seconds) const noexcept
    {
        const Vec2 target = agent.state.position + velocity * seconds;
        if (mesh.contains(target)) {
            return velocity;
        }
        // Binary-search the largest safe prefix. The initial position is guaranteed
        // contained, and this conservative clamp prevents a large side-step escaping
        // the mesh boundary.
        float low = 0.0f;
        float high = 1.0f;
        for (int iteration = 0; iteration < 20; ++iteration) {
            const float fraction = (low + high) * 0.5f;
            if (mesh.contains(agent.state.position + velocity * (seconds * fraction))) {
                low = fraction;
            } else {
                high = fraction;
            }
        }
        return velocity * low;
    }

    void substep(float seconds)
    {
        std::vector<Vec2> positions;
        std::vector<Vec2> desired;
        positions.reserve(agents.size());
        desired.reserve(agents.size());
        for (const AgentRecord& agent : agents) {
            positions.push_back(agent.state.position);
            desired.push_back(desired_velocity(agent, seconds));
        }

        std::vector<Vec2> chosen(agents.size());
        for (std::size_t index = 0; index < agents.size(); ++index) {
            const AgentRecord& agent = agents[index];
            const Vec2 preferred = desired[index];
            if (length(preferred) == 0.0f) {
                chosen[index] = contained_velocity(
                    agent, overlap_escape_velocity(index, positions, seconds), seconds);
                continue;
            }
            const Vec2 side = normalized(perpendicular_left(preferred)) * agent.state.max_speed;
            const float id_side = (agent.id.value & 1u) == 0u ? 1.0f : -1.0f;
            const std::array<Vec2, 7> candidates{
                preferred,
                capped(preferred + side * (0.75f * id_side), agent.state.max_speed),
                side * id_side,
                capped(preferred - side * (0.75f * id_side), agent.state.max_speed),
                preferred * 0.5f,
                preferred * -0.5f,
                Vec2{},
            };
            bool found_safe_velocity = false;
            for (const Vec2 candidate : candidates) {
                const Vec2 contained = contained_velocity(agent, candidate, seconds);
                if (safe_velocity(index, contained, positions, desired, seconds)) {
                    chosen[index] = contained;
                    found_safe_velocity = true;
                    break;
                }
            }
            if (!found_safe_velocity) {
                // A pre-existing overlap has no safe one-step candidate. Move apart
                // from the snapshot instead of indefinitely selecting zero velocity.
                chosen[index] = contained_velocity(agent,
                    overlap_escape_velocity(index, positions, seconds), seconds);
            }
        }

        for (std::size_t index = 0; index < agents.size(); ++index) {
            AgentRecord& agent = agents[index];
            agent.state.velocity = chosen[index];
            agent.state.position = agent.state.position + chosen[index] * seconds;
            advance_route(agent);
        }
    }
};

Simulation::Simulation(NavMesh mesh)
    : m_impl(std::make_unique<Impl>(std::move(mesh)))
{
}

Simulation::~Simulation() = default;
Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

[[nodiscard]] Result<AgentId> Simulation::add_agent(const AgentConfig& config)
{
    if (!m_impl) {
        return std::unexpected(fail(ErrorCode::InvalidArgument, "moved-from simulation"));
    }
    if (!geom::is_finite(config.position) || !std::isfinite(config.radius) ||
        !std::isfinite(config.max_speed) || config.radius <= 0.0f ||
        config.max_speed <= 0.0f) {
        return std::unexpected(fail(ErrorCode::InvalidArgument, "invalid agent configuration"));
    }
    if (!m_impl->mesh.contains(config.position)) {
        return std::unexpected(fail(ErrorCode::OutsideMesh, "agent position outside mesh"));
    }
    const Result<float> arrival = effective_arrival_radius(config.arrival_radius, config.radius);
    if (!arrival) {
        return std::unexpected(arrival.error());
    }
    if (config.goal && !geom::is_finite(*config.goal)) {
        return std::unexpected(fail(ErrorCode::InvalidArgument, "non-finite goal"));
    }
    if (config.goal && !m_impl->mesh.contains(*config.goal)) {
        return std::unexpected(fail(ErrorCode::OutsideMesh, "goal outside mesh"));
    }
    if (m_impl->next_id == 0u) {
        return std::unexpected(fail(ErrorCode::InvalidArgument, "agent id space exhausted"));
    }

    AgentRecord record;
    record.id = AgentId{m_impl->next_id++};
    record.state.position = config.position;
    record.state.radius = config.radius;
    record.state.max_speed = config.max_speed;
    m_impl->agents.push_back(std::move(record));
    AgentRecord& added = m_impl->agents.back();
    if (config.goal) {
        const Result<void> assigned = m_impl->assign_goal(added, *config.goal, config.arrival_radius);
        if (!assigned) {
            m_impl->agents.pop_back();
            return std::unexpected(assigned.error());
        }
    }
    return added.id;
}

[[nodiscard]] Result<void> Simulation::remove_agent(AgentId id)
{
    if (!m_impl) {
        return std::unexpected(fail(ErrorCode::InvalidArgument, "moved-from simulation"));
    }
    const auto found = std::ranges::find(m_impl->agents, id, &AgentRecord::id);
    if (found == m_impl->agents.end()) {
        return std::unexpected(fail(ErrorCode::NotFound, "unknown agent"));
    }
    m_impl->agents.erase(found);
    return {};
}

[[nodiscard]] Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius)
{
    if (!m_impl) {
        return std::unexpected(fail(ErrorCode::InvalidArgument, "moved-from simulation"));
    }
    AgentRecord* agent = m_impl->find(id);
    if (!agent) {
        return std::unexpected(fail(ErrorCode::NotFound, "unknown agent"));
    }
    return m_impl->assign_goal(*agent, goal, arrival_radius);
}

[[nodiscard]] Result<void> Simulation::clear_goal(AgentId id)
{
    if (!m_impl) {
        return std::unexpected(fail(ErrorCode::InvalidArgument, "moved-from simulation"));
    }
    AgentRecord* agent = m_impl->find(id);
    if (!agent) {
        return std::unexpected(fail(ErrorCode::NotFound, "unknown agent"));
    }
    agent->state.goal.reset();
    agent->state.velocity = {};
    agent->state.status = AgentStatus::Idle;
    agent->route.points.clear();
    agent->waypoint = 0;
    agent->arrival_radius = agent->state.radius;
    return {};
}

[[nodiscard]] Result<void> Simulation::step(float seconds)
{
    if (!std::isfinite(seconds) || seconds < 0.0f) {
        return std::unexpected(fail(ErrorCode::InvalidArgument, "invalid step duration"));
    }
    if (!m_impl) {
        return std::unexpected(fail(ErrorCode::InvalidArgument, "moved-from simulation"));
    }
    double remaining = seconds;
    while (remaining > 0.0) {
        const float slice = static_cast<float>(
            std::min(remaining, static_cast<double>(kSubstepSeconds)));
        m_impl->substep(slice);
        remaining -= static_cast<double>(slice);
    }
    return {};
}

[[nodiscard]] std::optional<AgentState> Simulation::agent(AgentId id) const noexcept
{
    if (!m_impl) {
        return std::nullopt;
    }
    const AgentRecord* record = m_impl->find(id);
    return record ? std::optional<AgentState>{record->state} : std::nullopt;
}

[[nodiscard]] std::size_t Simulation::agent_count() const noexcept
{
    return m_impl ? m_impl->agents.size() : 0u;
}

} // namespace vwmini
