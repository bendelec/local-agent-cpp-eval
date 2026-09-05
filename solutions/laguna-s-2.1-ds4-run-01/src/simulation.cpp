// VWmini agent simulation implementation (WP5/WP6).
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

inline constexpr float kEpsilon = 1e-4f;
inline constexpr float kEpsSq = kEpsilon * kEpsilon;

namespace {

float len_sq(Vec2 v) noexcept { return dot(v, v); }
Vec2 normalized_or_zero(Vec2 v) noexcept { return normalized(v); }
// normalized() already returns {0,0} for zero-length input per geometry.hpp contract.

// Project `candidate` back onto the mesh union by backtracking along the step that
// produced it from `previous`, using binary search on containment. Keeps agents
// inside without exposing internal triangle storage to Simulation.
Vec2 project_inside(const NavMesh& mesh, Vec2 previous, Vec2 candidate) noexcept
{
    if (mesh.contains(candidate)) {
        return candidate;
    }
    Vec2 lo = previous;
    Vec2 hi = candidate;
    if (!mesh.contains(lo)) {
        return previous;
    }
    for (int i = 0; i < 16; ++i) {
        const Vec2 mid = (lo + hi) * 0.5f;
        if (mesh.contains(mid)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return lo;
}

} // namespace

struct Simulation::Impl {
    struct LiveAgent {
        std::uint32_t id{};
        Vec2 position{};
        Vec2 velocity{};
        float radius{0.25f};
        float max_speed{1.4f};
        float arrival_radius{0.25f};
        std::optional<Vec2> goal{};
        AgentStatus status{AgentStatus::Idle};
        std::vector<Vec2> waypoints{};
    };

    NavMesh mesh;
    std::vector<LiveAgent> live;
    std::uint32_t next_id{1};

    explicit Impl(NavMesh m) : mesh(std::move(m)) {}

    LiveAgent* find(std::uint32_t id) noexcept
    {
        for (auto& a : live) {
            if (a.id == id) {
                return &a;
            }
        }
        return nullptr;
    }
    const LiveAgent* find(std::uint32_t id) const noexcept
    {
        return const_cast<Impl*>(this)->find(id);
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
    if (!std::isfinite(config.position.x) || !std::isfinite(config.position.y)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "agent position is non-finite"});
    }
    if (!(config.radius > 0.0f)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "agent radius must be positive"});
    }
    if (!(config.max_speed >= 0.0f)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "agent max_speed must be non-negative"});
    }
    if (!m_impl->mesh.contains(config.position)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "agent position is outside the navmesh"});
    }

    Impl::LiveAgent agent{};
    agent.id = m_impl->next_id++;
    if (m_impl->next_id == 0) {
        m_impl->next_id = 1;
    }
    agent.position = config.position;
    agent.velocity = {};
    agent.radius = config.radius;
    agent.max_speed = config.max_speed;
    agent.arrival_radius = (config.arrival_radius >= 0.0f) ? config.arrival_radius : config.radius;
    // Default status is Idle (from LiveAgent's initializer); set_goal below upgrades it.

    const std::uint32_t assigned_id = agent.id;
    m_impl->live.push_back(std::move(agent));

    if (config.goal.has_value()) {
        const Result<void> g = set_goal(AgentId{assigned_id}, *config.goal, config.arrival_radius);
        if (!g && g.error().code != ErrorCode::NoPath) {
            // Roll back the partially-initialized agent on hard failure.
            auto& live = m_impl->live;
            for (auto it = live.begin(); it != live.end(); ++it) {
                if (it->id == assigned_id) {
                    live.erase(it);
                    break;
                }
            }
            return std::unexpected(g.error());
        }
    }

    return AgentId{assigned_id};
}

Result<void> Simulation::remove_agent(AgentId id)
{
    auto& live = m_impl->live;
    for (auto it = live.begin(); it != live.end(); ++it) {
        if (it->id == id.value) {
            live.erase(it);
            return {};
        }
    }
    return std::unexpected(Error{ErrorCode::NotFound, "unknown agent id"});
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius)
{
    if (!std::isfinite(goal.x) || !std::isfinite(goal.y)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "goal point is non-finite"});
    }
    Impl::LiveAgent* a = m_impl->find(id.value);
    if (!a) {
        return std::unexpected(Error{ErrorCode::NotFound, "unknown agent id"});
    }
    if (!m_impl->mesh.contains(goal)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "goal point is outside the navmesh"});
    }

    a->arrival_radius = (arrival_radius >= 0.0f) ? arrival_radius : a->radius;
    a->goal = goal;

    const Result<Path> route = find_path(m_impl->mesh, a->position, goal);
    if (!route) {
        if (route.error().code == ErrorCode::NoPath) {
            a->status = AgentStatus::NoPath;
            a->waypoints.clear();
            a->velocity = {};
            return {};
        }
        return std::unexpected(route.error());
    }

    a->waypoints.clear();
    bool skipped_start = false;
    for (const Vec2 p : route->points) {
        if (!skipped_start && len_sq(p - a->position) <= kEpsSq) {
            skipped_start = true;
            continue;
        }
        a->waypoints.push_back(p);
    }
    a->status = AgentStatus::Moving;
    return {};
}

Result<void> Simulation::clear_goal(AgentId id)
{
    Impl::LiveAgent* a = m_impl->find(id.value);
    if (!a) {
        return std::unexpected(Error{ErrorCode::NotFound, "unknown agent id"});
    }
    a->goal.reset();
    a->status = AgentStatus::Idle;
    a->velocity = {};
    a->waypoints.clear();
    return {};
}

// (moving-agent integration lives inline in step() below)

Result<void> Simulation::step(float seconds)
{
    if (!(seconds >= 0.0f) || !std::isfinite(seconds)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "step duration must be finite and non-negative"});
    }
    if (seconds == 0.0f) {
        return {};
    }

    auto& live = m_impl->live;
    if (live.empty()) {
        return {};
    }

    // Symmetric positional separation among overlapping agents (deterministic order).
    // Uses pre-integration positions so avoidance stays stable within this substep.
    for (std::size_t i = 0; i < live.size(); ++i) {
        for (std::size_t j = i + 1; j < live.size(); ++j) {
            const Vec2 delta = live[j].position - live[i].position;
            const float rsum = live[i].radius + live[j].radius;
            const float d2 = len_sq(delta);
            if (d2 >= rsum * rsum || d2 <= kEpsSq) {
                continue;
            }
            const Vec2 nrm = delta * (1.0f / std::sqrt(d2));
            const float pen = rsum - std::sqrt(d2);
            const Vec2 push = nrm * (pen * 0.5f);
            live[i].position = live[i].position - push;
            live[j].position = live[j].position + push;
        }
    }

    // Integrate using each agent's own steering basis with clamped speed and clamping.
    auto advance = [this](Impl::LiveAgent& a, float dt) {
        if (a.waypoints.empty() && !a.goal.has_value()) {
            a.velocity = {};
            return;
        }
        const Vec2 target = a.waypoints.empty() ? *a.goal : a.waypoints.front();
        Vec2 to_target = target - a.position;
        const float dist_to_target = length(to_target);

        Vec2 desired = normalized_or_zero(to_target) * a.max_speed;
        Vec2 vel = desired + a.velocity * 0.5f;
        const float sp = length(vel);
        if (sp > a.max_speed && sp > 0.0f) {
            vel = vel * (a.max_speed / sp);
        }

        const Vec2 prev_pos = a.position;
        Vec2 next_pos = a.position + vel * dt;
        const float move_budget = a.max_speed * dt;
        if (dist_to_target <= move_budget + kEpsilon || dot(next_pos - a.position, to_target) < 0.0f) {
            next_pos = a.position + normalized_or_zero(to_target) *
                                    std::min(move_budget, dist_to_target);
        }
        a.position = project_inside(m_impl->mesh, prev_pos, next_pos);
        a.velocity = (dt > 0.0f) ? (a.position - prev_pos) * (1.0f / dt) : Vec2{};

        if (length(target - a.position) <= a.arrival_radius) {
            if (!a.waypoints.empty()) {
                a.waypoints.erase(a.waypoints.begin());
            } else if (a.goal.has_value()) {
                a.status = AgentStatus::Reached;
                a.position = *a.goal;
                a.velocity = {};
                a.goal.reset();
                a.waypoints.clear();
            }
        }
    };

    for (std::size_t i = 0; i < live.size(); ++i) {
        Impl::LiveAgent& a = live[i];
        if (a.status != AgentStatus::Moving) {
            a.velocity = {};
            continue;
        }
        advance(a, seconds);
    }

    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept
{
    const Impl::LiveAgent* a = m_impl->find(id.value);
    if (!a) {
        return std::nullopt;
    }
    AgentState state{};
    state.position = a->position;
    state.velocity = a->velocity;
    state.radius = a->radius;
    state.max_speed = a->max_speed;
    state.goal = a->goal;
    state.status = a->status;
    return state;
}

std::size_t Simulation::agent_count() const noexcept
{
    return m_impl ? m_impl->live.size() : 0;
}

} // namespace vwmini
