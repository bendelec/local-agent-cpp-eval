#include <vwmini/simulation.hpp>

#include <vwmini/detail.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace vwmini {

struct Simulation::Impl {
    NavMesh mesh;
    struct Agent {
        bool alive = false;
        Vec2 position{};
        Vec2 velocity{};
        float radius = 0.0f;
        float max_speed = 0.0f;
        std::optional<Vec2> goal{};
        AgentStatus status = AgentStatus::Idle;
        std::vector<Vec2> route;
        std::size_t target_index = 0;
        float arrival_radius = -1.0f;
        float effective_arrival_radius = 0.0f;
    };
    std::vector<Agent> agents;

    // Apply an in-mesh goal: compute route and transition state (SIM-007).
    static Result<void> assign_goal(Impl& impl, Agent& a, Vec2 goal_pos,
                                    float arrival_radius);

    explicit Impl(NavMesh mesh_) : mesh(std::move(mesh_)) {}
};

Simulation::Simulation(NavMesh mesh) : m_impl(std::make_unique<Impl>(std::move(mesh)))
{
}

Simulation::~Simulation() = default;
Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

namespace {

// Resolve the arrival-radius rule (SIM-006). Returns false on invalid input.
bool resolve_arrival(float arrival_radius, float radius, float& out)
{
    if (!std::isfinite(arrival_radius)) {
        return false;
    }
    if (arrival_radius == -1.0f) {
        out = radius;
        return true;
    }
    if (arrival_radius < 0.0f) {
        return false; // any other negative is invalid
    }
    out = arrival_radius;
    return true;
}

bool validate_agent_scalars(const AgentConfig& cfg)
{
    if (!detail::finite(cfg.position) || !detail::finite(cfg.radius) ||
        !detail::finite(cfg.max_speed)) {
        return false;
    }
    if (cfg.radius <= 0.0f || cfg.max_speed <= 0.0f) {
        return false;
    }
    float tmp = 0.0f;
    if (!resolve_arrival(cfg.arrival_radius, cfg.radius, tmp)) {
        return false;
    }
    if (cfg.goal.has_value() && !detail::finite(cfg.goal.value())) {
        return false;
    }
    return true;
}

// Scale v back along its direction so that pos + sub_dt*v stays in the mesh;
// returns the scale in [0,1].
float clamp_into_mesh(const NavMesh& mesh, Vec2 pos, Vec2 v, float sub_dt)
{
    const Vec2 end = pos + v * sub_dt;
    if (mesh.contains(end)) {
        return 1.0f;
    }
    float lo = 0.0f, hi = 1.0f;
    for (int _ = 0; _ < 16; ++_) {
        const float mid = 0.5f * (lo + hi);
        if (mesh.contains(pos + v * (sub_dt * mid))) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return lo;
}

} // namespace

// Apply an in-mesh goal: compute route and transition state (SIM-007).
Result<void> Simulation::Impl::assign_goal(Impl& impl, Agent& a, Vec2 goal_pos,
                                           float arrival_radius)
{
    float eff = 0.0f;
    if (!resolve_arrival(arrival_radius, a.radius, eff)) {
        return std::unexpected(
            Error{ErrorCode::InvalidArgument, "invalid arrival radius"});
    }
    a.goal = goal_pos;
    a.arrival_radius = arrival_radius;
    a.effective_arrival_radius = eff;

    Result<Path> path_or = find_path(impl.mesh, a.position, goal_pos);
    if (!path_or) {
        const Error& e = path_or.error();
        if (e.code == ErrorCode::NoPath) {
            a.status = AgentStatus::NoPath;
            a.velocity = Vec2{0.0f, 0.0f};
            a.route.clear();
            a.target_index = 0;
            return {};
        }
        return std::unexpected(e); // OutsideMesh (defensive; callers validate in-mesh)
    }

    Path path = std::move(*path_or);
    a.route = std::move(path.points);

    if (a.route.size() < 2 ||
        length(a.position - goal_pos) <= a.effective_arrival_radius) {
        a.status = AgentStatus::Reached;
        a.velocity = Vec2{0.0f, 0.0f};
        a.target_index = a.route.empty() ? 0u : (a.route.size() - 1);
        return {};
    }

    a.status = AgentStatus::Moving;
    a.velocity = Vec2{0.0f, 0.0f};
    a.target_index = 1;
    return {};
}

Result<AgentId> Simulation::add_agent(const AgentConfig& config)
{
    if (!validate_agent_scalars(config)) {
        return std::unexpected(
            Error{ErrorCode::InvalidArgument, "invalid agent configuration"});
    }
    if (!m_impl->mesh.contains(config.position)) {
        return std::unexpected(
            Error{ErrorCode::OutsideMesh, "agent position is outside the mesh"});
    }

    const std::uint32_t id = static_cast<std::uint32_t>(m_impl->agents.size()) + 1u;
    auto& a = m_impl->agents.emplace_back();
    a.alive = true;
    a.position = config.position;
    a.radius = config.radius;
    a.max_speed = config.max_speed;
    a.velocity = Vec2{0.0f, 0.0f};
    a.arrival_radius = config.arrival_radius;

    if (config.goal.has_value()) {
        const Vec2 g = config.goal.value();
        if (!m_impl->mesh.contains(g)) {
            return std::unexpected(
                Error{ErrorCode::OutsideMesh, "agent goal is outside the mesh"});
        }
        Result<void> r = Impl::assign_goal(*m_impl, a, g, config.arrival_radius);
        if (!r) {
            m_impl->agents.pop_back();
            return std::unexpected(r.error());
        }
    }
    return AgentId{id};
}

Result<void> Simulation::remove_agent(AgentId id)
{
    if (id.value == 0 || id.value > m_impl->agents.size() ||
        !m_impl->agents[id.value - 1].alive) {
        return std::unexpected(
            Error{ErrorCode::NotFound, "unknown or removed agent id"});
    }
    m_impl->agents[id.value - 1].alive = false;
    return {};
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius)
{
    if (id.value == 0 || id.value > m_impl->agents.size() ||
        !m_impl->agents[id.value - 1].alive) {
        return std::unexpected(
            Error{ErrorCode::NotFound, "unknown or removed agent id"});
    }
    if (!detail::finite(goal)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite goal"});
    }
    float tmp = 0.0f;
    if (!resolve_arrival(arrival_radius, m_impl->agents[id.value - 1].radius, tmp)) {
        return std::unexpected(
            Error{ErrorCode::InvalidArgument, "invalid arrival radius"});
    }
    auto& a = m_impl->agents[id.value - 1];
    if (!m_impl->mesh.contains(goal)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "goal is outside the mesh"});
    }
    return Impl::assign_goal(*m_impl, a, goal, arrival_radius);
}

Result<void> Simulation::clear_goal(AgentId id)
{
    if (id.value == 0 || id.value > m_impl->agents.size() ||
        !m_impl->agents[id.value - 1].alive) {
        return std::unexpected(
            Error{ErrorCode::NotFound, "unknown or removed agent id"});
    }
    auto& a = m_impl->agents[id.value - 1];
    a.goal.reset();
    a.route.clear();
    a.target_index = 0;
    a.velocity = Vec2{0.0f, 0.0f};
    a.status = AgentStatus::Idle;
    return {};
}

Result<void> Simulation::step(float seconds)
{
    if (!std::isfinite(seconds) || seconds < 0.0f) {
        return std::unexpected(
            Error{ErrorCode::InvalidArgument, "invalid step duration"});
    }
    if (seconds == 0.0f) {
        return {};
    }

    const NavMesh& mesh = m_impl->mesh;
    std::vector<Impl::Agent*> live;
    for (auto& a : m_impl->agents) {
        if (a.alive) {
            live.push_back(&a);
        }
    }
    if (live.empty()) {
        return {};
    }

    constexpr float target_substep = 0.05f;
    const std::size_t n_sub = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(seconds / target_substep)));
    const float sub_dt = seconds / static_cast<float>(n_sub);
    const float sep_bias = 1e-3f;

    struct Snap {
        Vec2 pos;
        Vec2 vel;
        Vec2 dest;
        float radius;
        float max_speed;
        bool moving;
    };

    for (std::size_t s = 0; s < n_sub; ++s) {
        // 1. Snapshot positions/velocities (read before any write).
        std::vector<Snap> snap;
        snap.reserve(live.size());
        for (auto* a : live) {
            Vec2 dest = a->position;
            if (a->status == AgentStatus::Moving && a->route.size() >= 2 &&
                a->target_index < a->route.size()) {
                dest = a->route[a->target_index];
            }
            snap.push_back({a->position, a->velocity, dest, a->radius,
                            a->max_speed, a->status == AgentStatus::Moving});
        }

        // 2. Desired velocity: head to current waypoint, capped, arrive on it.
        std::vector<Vec2> v(snap.size(), Vec2{0.0f, 0.0f});
        for (std::size_t i = 0; i < snap.size(); ++i) {
            if (!snap[i].moving) {
                continue;
            }
            const Vec2 to_dest = snap[i].dest - snap[i].pos;
            const float dist = length(to_dest);
            if (dist > 0.0f) {
                const Vec2 dir = to_dest * (1.0f / dist);
                const float speed = std::min(snap[i].max_speed, dist / sub_dt);
                v[i] = dir * speed;
            }
        }

        // 3. Reciprocal collision avoidance (snapshot-based, simultaneous).
        for (int pass = 0; pass < 3; ++pass) {
            for (std::size_t i = 0; i < snap.size(); ++i) {
                for (std::size_t j = i + 1; j < snap.size(); ++j) {
                    const Vec2 diff = snap[j].pos - snap[i].pos;
                    const float dist = length(diff);
                    if (dist <= 0.0f) {
                        continue;
                    }
                    const Vec2 n = diff * (1.0f / dist);
                    const float rsum = snap[i].radius + snap[j].radius;
                    Vec2 rel = v[j] - v[i];
                    const float approach = dot(rel, n);
                    float target_rel = 0.0f;
                    if (dist < rsum) {
                        target_rel = (rsum - dist) / sub_dt + sep_bias;
                    }
                    if (approach < target_rel) {
                        const float needed = target_rel - approach;
                        v[i] = v[i] - n * (0.5f * needed);
                        v[j] = v[j] + n * (0.5f * needed);
                    }
                }
            }
            // Cap each velocity to its max speed.
            for (std::size_t i = 0; i < snap.size(); ++i) {
                const float L = length(v[i]);
                if (L > snap[i].max_speed) {
                    v[i] = v[i] * (snap[i].max_speed / L);
                }
            }
        }

        // 4. Keep every agent inside the mesh.
        for (std::size_t i = 0; i < snap.size(); ++i) {
            const float scale = clamp_into_mesh(mesh, snap[i].pos, v[i], sub_dt);
            v[i] = v[i] * scale;
        }

        // 5. Apply motion with waypoint-overshoot protection, then update state.
        for (std::size_t k = 0; k < live.size(); ++k) {
            auto* a = live[k];
            Vec2 delta = v[k] * sub_dt;

            if (a->status == AgentStatus::Moving) {
                const Vec2 dest = a->route[a->target_index];
                const Vec2 to_dest = dest - a->position;
                const float dist = length(to_dest);
                if (dist > 0.0f) {
                    const Vec2 dir = to_dest * (1.0f / dist);
                    const float along = dot(delta, dir);
                    if (along > dist) {
                        const Vec2 perp = delta - dir * along;
                        delta = perp + dir * dist; // stop exactly at the waypoint
                    }
                }
            }

            Vec2 new_pos = a->position + delta;
            if (detail::finite(new_pos) && !mesh.contains(new_pos)) {
                float lo = 0.0f, hi = 1.0f;
                for (int _ = 0; _ < 16; ++_) {
                    const float mid = 0.5f * (lo + hi);
                    if (mesh.contains(a->position + delta * mid)) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                new_pos = a->position + delta * lo;
            }

            a->velocity = (sub_dt > 0.0f) ? delta * (1.0f / sub_dt)
                                        : Vec2{0.0f, 0.0f};
            a->position = new_pos;

            // Advance waypoint when reached.
            if (a->status == AgentStatus::Moving && a->route.size() >= 2 &&
                a->target_index < a->route.size()) {
                const Vec2 w = a->route[a->target_index];
                if (length(a->position - w) <= detail::epsilon) {
                    a->target_index += 1;
                }
            }
            // Arrival at the goal.
            if (a->status == AgentStatus::Moving && a->goal.has_value()) {
                if (length(a->position - a->goal.value()) <=
                    a->effective_arrival_radius) {
                    a->status = AgentStatus::Reached;
                    a->velocity = Vec2{0.0f, 0.0f};
                }
            }
        }
    }

    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept
{
    if (id.value == 0 || id.value > m_impl->agents.size() ||
        !m_impl->agents[id.value - 1].alive) {
        return std::nullopt;
    }
    const auto& a = m_impl->agents[id.value - 1];
    AgentState s;
    s.position = a.position;
    s.velocity = a.velocity;
    s.radius = a.radius;
    s.max_speed = a.max_speed;
    s.goal = a.goal;
    s.status = a.status;
    return s;
}

std::size_t Simulation::agent_count() const noexcept
{
    std::size_t count = 0;
    for (const auto& a : m_impl->agents) {
        if (a.alive) {
            count += 1;
        }
    }
    return count;
}

} // namespace vwmini
