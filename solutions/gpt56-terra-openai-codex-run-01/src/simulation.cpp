#include <vwmini/simulation.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace vwmini {
namespace {

constexpr float kEpsilon = 1.0e-4F;
constexpr float kMaximumSubstep = 0.02F;

[[nodiscard]] bool finite(Vec2 value) noexcept
{
    return std::isfinite(value.x) && std::isfinite(value.y);
}

[[nodiscard]] bool valid_arrival_radius(float value) noexcept
{
    return std::isfinite(value) && (value >= 0.0F || value == -1.0F);
}

[[nodiscard]] float distance(Vec2 a, Vec2 b) noexcept
{
    return length(a - b);
}

[[nodiscard]] Vec2 perpendicular(Vec2 value) noexcept
{
    return {-value.y, value.x};
}

[[nodiscard]] Error invalid_argument(const char* message)
{
    return {ErrorCode::InvalidArgument, message};
}

[[nodiscard]] Error not_found()
{
    return {ErrorCode::NotFound, "agent id was not found"};
}

} // namespace

struct Simulation::Impl {
    struct Agent {
        AgentId id;
        Vec2 position;
        Vec2 velocity{};
        float radius;
        float max_speed;
        std::optional<Vec2> goal;
        float arrival_radius{};
        AgentStatus status{AgentStatus::Idle};
        std::vector<Vec2> route;
        std::size_t waypoint{};
    };

    NavMesh mesh;
    std::vector<Agent> agents;
    std::uint32_t next_id{1};

    explicit Impl(NavMesh input_mesh)
        : mesh(std::move(input_mesh))
    {
    }

    [[nodiscard]] Agent* find(AgentId id) noexcept
    {
        const auto found = std::find_if(agents.begin(), agents.end(),
                                        [id](const Agent& agent) { return agent.id == id; });
        return found == agents.end() ? nullptr : &*found;
    }

    [[nodiscard]] const Agent* find(AgentId id) const noexcept
    {
        const auto found = std::find_if(agents.begin(), agents.end(),
                                        [id](const Agent& agent) { return agent.id == id; });
        return found == agents.end() ? nullptr : &*found;
    }
};

struct SimulationAccess {

static void stop(Simulation::Impl::Agent& agent) noexcept
{
    agent.velocity = {};
}

static void set_reached(Simulation::Impl::Agent& agent) noexcept
{
    agent.status = AgentStatus::Reached;
    stop(agent);
    agent.route.clear();
    agent.waypoint = 0;
}

static void set_no_path(Simulation::Impl::Agent& agent) noexcept
{
    agent.status = AgentStatus::NoPath;
    stop(agent);
    agent.route.clear();
    agent.waypoint = 0;
}

static void apply_route(Simulation::Impl::Agent& agent, const Path& path) noexcept
{
    agent.route = path.points;
    agent.waypoint = agent.route.size() > 1 ? 1 : 0;
    agent.status = AgentStatus::Moving;
}

[[nodiscard]] static Result<void> assign_goal(Simulation::Impl& impl, Simulation::Impl::Agent& agent,
                                       Vec2 goal, float arrival_radius)
{
    const float effective_radius = arrival_radius == -1.0F ? agent.radius : arrival_radius;
    const auto route = find_path(impl.mesh, agent.position, goal);
    agent.goal = goal;
    agent.arrival_radius = effective_radius;
    if (!route) {
        if (route.error().code == ErrorCode::NoPath) {
            set_no_path(agent);
            return {};
        }
        return std::unexpected(route.error());
    }
    if (distance(agent.position, goal) <= effective_radius) {
        set_reached(agent);
        return {};
    }
    apply_route(agent, *route);
    return {};
}

[[nodiscard]] static Vec2 preferred_velocity(const Simulation::Impl::Agent& agent,
                                             float seconds) noexcept
{
    if (agent.status != AgentStatus::Moving || agent.waypoint >= agent.route.size()) {
        return {};
    }
    const Vec2 offset = agent.route[agent.waypoint] - agent.position;
    const float remaining = length(offset);
    if (remaining <= kEpsilon) {
        return {};
    }
    // Reach a close waypoint exactly instead of allowing a fixed substep to pass it.
    const float speed = std::min(agent.max_speed, remaining / seconds);
    return normalized(offset) * speed;
}

[[nodiscard]] static Vec2 bounded_velocity(Vec2 velocity, float max_speed) noexcept
{
    const float velocity_length = length(velocity);
    if (!std::isfinite(velocity_length) || velocity_length == 0.0F) {
        return {};
    }
    return velocity_length > max_speed ? velocity * (max_speed / velocity_length) : velocity;
}

[[nodiscard]] static Vec2 choose_velocity(const Simulation::Impl& impl, std::size_t index,
                                   const std::vector<Vec2>& positions,
                                   const std::vector<Vec2>& velocities, float seconds) noexcept
{
    const auto& agent = impl.agents[index];
    Vec2 chosen = preferred_velocity(agent, seconds);
    if (chosen == Vec2{}) {
        return {};
    }

    // A deterministic reciprocal separation correction based solely on this substep snapshot.
    for (std::size_t other_index = 0; other_index < impl.agents.size(); ++other_index) {
        if (other_index == index) {
            continue;
        }
        const auto& other = impl.agents[other_index];
        const Vec2 difference = positions[index] - positions[other_index];
        const float separation = length(difference);
        const float required = agent.radius + other.radius;
        const Vec2 relative_motion = chosen - velocities[other_index];
        const float predicted = length(difference + relative_motion * seconds);
        if (predicted >= required && separation >= required) {
            continue;
        }

        Vec2 away;
        if (separation > kEpsilon) {
            away = difference * (1.0F / separation);
        } else {
            // Id order gives initially coincident agents opposite, reproducible directions.
            away = agent.id.value < other.id.value ? Vec2{-1.0F, 0.0F} : Vec2{1.0F, 0.0F};
        }
        const float pressure = std::clamp((required - std::min(predicted, separation))
                                              / std::max(required, kEpsilon),
                                          0.0F, 1.0F);
        chosen = chosen + away * (agent.max_speed * (0.8F + pressure));
        if (cross(chosen, away) == 0.0F && dot(chosen, away) < 0.0F) {
            const Vec2 side = perpendicular(away);
            chosen = chosen + side * (agent.id.value < other.id.value ? 0.35F : -0.35F)
                * agent.max_speed;
        }
    }
    return bounded_velocity(chosen, agent.max_speed);
}

[[nodiscard]] static bool direct_motion_is_contained(const NavMesh& mesh, Vec2 start,
                                                      Vec2 end)
{
    const auto path = find_path(mesh, start, end);
    return path && path->points.size() == 2 && path->points[0] == start && path->points[1] == end;
}

static void resolve_predicted_overlaps(const Simulation::Impl& impl,
                                       const std::vector<Vec2>& positions,
                                       std::vector<Vec2>& decisions, float seconds) noexcept
{
    // Resolve choices against the same position snapshot.  A few fixed passes make pair order
    // deterministic while retaining the simultaneous-observation rule.
    for (int pass = 0; pass < 4; ++pass) {
        for (std::size_t first = 0; first < impl.agents.size(); ++first) {
            for (std::size_t second = first + 1; second < impl.agents.size(); ++second) {
                const float required = impl.agents[first].radius + impl.agents[second].radius;
                const Vec2 current = positions[first] - positions[second];
                const Vec2 predicted = current + (decisions[first] - decisions[second]) * seconds;
                const float predicted_distance = length(predicted);
                if (predicted_distance >= required) {
                    continue;
                }
                Vec2 direction = predicted_distance > kEpsilon ? normalized(predicted)
                    : (length(current) > kEpsilon ? normalized(current)
                       : (impl.agents[first].id.value < impl.agents[second].id.value
                              ? Vec2{-1.0F, 0.0F}
                              : Vec2{1.0F, 0.0F}));
                const Vec2 target = direction * required;
                const Vec2 correction = (target - predicted) * (0.5F / seconds);
                decisions[first] = bounded_velocity(decisions[first] + correction,
                                                    impl.agents[first].max_speed);
                decisions[second] = bounded_velocity(decisions[second] - correction,
                                                     impl.agents[second].max_speed);

                const Vec2 corrected = current + (decisions[first] - decisions[second]) * seconds;
                if (length(corrected) < required) {
                    // Bounded correction had insufficient radial authority; a safe local yield
                    // is preferable to allowing a predicted disc overlap.
                    decisions[first] = direction * impl.agents[first].max_speed;
                    decisions[second] = direction * -impl.agents[second].max_speed;
                }
            }
        }
    }
}

[[nodiscard]] static Vec2 safe_displacement(const Simulation::Impl& impl,
                                             const Simulation::Impl::Agent& agent, Vec2 velocity,
                                             float seconds)
{
    const Vec2 displacement = velocity * seconds;
    if (length(displacement) == 0.0F) {
        return {};
    }
    if (direct_motion_is_contained(impl.mesh, agent.position, agent.position + displacement)) {
        return displacement;
    }
    // Bisection also checks the complete segment, so steering cannot skip through a concavity.
    float low = 0.0F;
    float high = 1.0F;
    for (int iteration = 0; iteration < 20; ++iteration) {
        const float middle = (low + high) * 0.5F;
        if (direct_motion_is_contained(impl.mesh, agent.position,
                                       agent.position + displacement * middle)) {
            low = middle;
        } else {
            high = middle;
        }
    }
    return displacement * low;
}

static void advance_one_substep(Simulation::Impl& impl, float seconds)
{
    const std::size_t count = impl.agents.size();
    std::vector<Vec2> positions;
    std::vector<Vec2> velocities;
    positions.reserve(count);
    velocities.reserve(count);
    for (const auto& agent : impl.agents) {
        positions.push_back(agent.position);
        velocities.push_back(agent.velocity);
    }

    std::vector<Vec2> decisions(count);
    for (std::size_t index = 0; index < count; ++index) {
        decisions[index] = choose_velocity(impl, index, positions, velocities, seconds);
    }
    resolve_predicted_overlaps(impl, positions, decisions, seconds);
    for (std::size_t index = 0; index < count; ++index) {
        auto& agent = impl.agents[index];
        if (agent.status != AgentStatus::Moving) {
            stop(agent);
            continue;
        }
        const Vec2 displacement = safe_displacement(impl, agent, decisions[index], seconds);
        agent.position = agent.position + displacement;
        agent.velocity = seconds > 0.0F ? displacement * (1.0F / seconds) : Vec2{};

        while (agent.waypoint < agent.route.size()
               && distance(agent.position, agent.route[agent.waypoint]) <= kEpsilon) {
            ++agent.waypoint;
        }
        if (agent.goal && distance(agent.position, *agent.goal) <= agent.arrival_radius) {
            set_reached(agent);
        } else if (agent.waypoint >= agent.route.size()) {
            // A route endpoint is the goal.  Stop instead of overshooting it on a large substep.
            if (agent.goal) {
                agent.position = *agent.goal;
                set_reached(agent);
            }
        }
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

Result<AgentId> Simulation::add_agent(const AgentConfig& config)
{
    if (!finite(config.position) || !std::isfinite(config.radius) || !std::isfinite(config.max_speed)
        || !std::isfinite(config.arrival_radius)
        || (config.goal && !finite(*config.goal)) || config.radius <= 0.0F
        || config.max_speed <= 0.0F || !valid_arrival_radius(config.arrival_radius)) {
        return std::unexpected(invalid_argument("agent configuration is invalid"));
    }
    if (!m_impl->mesh.contains(config.position)
        || (config.goal && !m_impl->mesh.contains(*config.goal))) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "agent position or goal is outside the mesh"});
    }
    if (m_impl->next_id == 0) {
        return std::unexpected(invalid_argument("agent identifier space is exhausted"));
    }

    Impl::Agent candidate{AgentId{m_impl->next_id++}, config.position, {}, config.radius,
                          config.max_speed, std::nullopt, 0.0F, AgentStatus::Idle, {}, 0};
    if (config.goal) {
        const auto assigned = SimulationAccess::assign_goal(*m_impl, candidate, *config.goal, config.arrival_radius);
        if (!assigned) {
            return std::unexpected(assigned.error());
        }
    }
    m_impl->agents.push_back(std::move(candidate));
    return m_impl->agents.back().id;
}

Result<void> Simulation::remove_agent(AgentId id)
{
    const auto found = std::find_if(m_impl->agents.begin(), m_impl->agents.end(),
                                    [id](const Impl::Agent& agent) { return agent.id == id; });
    if (found == m_impl->agents.end()) {
        return std::unexpected(not_found());
    }
    m_impl->agents.erase(found);
    return {};
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius)
{
    Impl::Agent* target = m_impl->find(id);
    if (!target) {
        return std::unexpected(not_found());
    }
    if (!finite(goal) || !valid_arrival_radius(arrival_radius)) {
        return std::unexpected(invalid_argument("goal or arrival radius is invalid"));
    }
    if (!m_impl->mesh.contains(goal)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "goal is outside the mesh"});
    }

    // Find the route first, so all externally-invalid requests leave the live state intact.
    const float effective_radius = arrival_radius == -1.0F ? target->radius : arrival_radius;
    const auto route = find_path(m_impl->mesh, target->position, goal);
    if (!route && route.error().code != ErrorCode::NoPath) {
        return std::unexpected(route.error());
    }
    target->goal = goal;
    target->arrival_radius = effective_radius;
    if (!route) {
        SimulationAccess::set_no_path(*target);
    } else if (distance(target->position, goal) <= effective_radius) {
        SimulationAccess::set_reached(*target);
    } else {
        SimulationAccess::apply_route(*target, *route);
    }
    return {};
}

Result<void> Simulation::clear_goal(AgentId id)
{
    Impl::Agent* target = m_impl->find(id);
    if (!target) {
        return std::unexpected(not_found());
    }
    target->goal.reset();
    target->arrival_radius = 0.0F;
    target->route.clear();
    target->waypoint = 0;
    target->status = AgentStatus::Idle;
    SimulationAccess::stop(*target);
    return {};
}

Result<void> Simulation::step(float seconds)
{
    if (!std::isfinite(seconds) || seconds < 0.0F) {
        return std::unexpected(invalid_argument("step duration is invalid"));
    }
    if (seconds == 0.0F) {
        return {};
    }
    float remaining = seconds;
    while (remaining > 0.0F) {
        const float substep = std::min(remaining, kMaximumSubstep);
        SimulationAccess::advance_one_substep(*m_impl, substep);
        remaining -= substep;
    }
    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept
{
    const Impl::Agent* found = m_impl->find(id);
    if (!found) {
        return std::nullopt;
    }
    return AgentState{found->position, found->velocity, found->radius, found->max_speed,
                      found->goal, found->status};
}

std::size_t Simulation::agent_count() const noexcept
{
    return m_impl->agents.size();
}

} // namespace vwmini
