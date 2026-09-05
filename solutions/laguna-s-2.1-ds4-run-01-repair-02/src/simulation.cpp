// VWmini agent simulation implementation: validated lifecycle + snapshot-based bounded
// local avoidance. Steering selects feasible per-substep velocities honoring the speed
// budget, disc-disc non-overlap horizon, and mesh-containment invariants.
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
inline constexpr float kPi = 3.14159265358979323846f;

// Substep cap chosen so a max-speed agent cannot tunnel through reflex corners or skip
// past another disc between containment projections.
inline constexpr float kMaxSubStep = 0.05f;
// Defensive upper bound on substep count per step() call prevents pathological durations
// (e.g. FLT_MAX) from looping forever once dynamics quiesce.
inline constexpr int kMaxSubsteps = 1 << 20;

namespace {

bool finite_vec(Vec2 p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

float len_sq(Vec2 v) noexcept {
    return dot(v, v);
}

Vec2 desired_unit(Vec2 v) noexcept {
    return normalized(v);
} // zero input -> {0,0}

// Deterministic unit vector derived from an integer seed (golden-ratio hash). Used only
// to break coincident-agent symmetry without introducing nondeterminism.
Vec2 escape_dir(std::uint32_t seed) noexcept {
    std::uint32_t h = seed;
    h ^= h >> 13;
    h *= 2654435761u;
    h ^= h >> 16;
    const float frac = static_cast<float>(h) * (1.0f / 4294967296.0f);
    const float ang = 2.0f * kPi * frac;
    return Vec2{std::cos(ang), std::sin(ang)};
}

// Project `candidate` back onto the mesh union by backtracking along the step produced
// from `previous`, binary-searching on containment. Keeps agents inside without exposing
// triangle storage. Displacement stays within |next-prev| so it never exceeds the budget.
Vec2 project_inside(const NavMesh& mesh, Vec2 previous, Vec2 candidate) noexcept {
    if (mesh.contains(candidate)) {
        return candidate;
    }
    if (!mesh.contains(previous)) {
        return previous;
    }
    Vec2 lo = previous;
    Vec2 hi = candidate;
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

// Resolve -1 arrival-radius sentinel into the effective value, validating finiteness and
// rejecting every other negative radius as InvalidArgument. Zero/-0/positive are valid.
Result<void> resolve_arrival(float requested, float radius, float& out) noexcept {
    if (!std::isfinite(requested)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "arrival_radius is non-finite"});
    }
    if (requested == -1.0f) {
        out = radius;
        return {};
    }
    if (!(requested >= 0.0f)) {
        return std::unexpected(
            Error{ErrorCode::InvalidArgument, "arrival_radius must be >= 0 or exactly -1"});
    }
    out = requested;
    return {};
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

    LiveAgent* find(std::uint32_t id) noexcept {
        for (auto& a : live) {
            if (a.id == id) {
                return &a;
            }
        }
        return nullptr;
    }
    const LiveAgent* find(std::uint32_t id) const noexcept {
        return const_cast<Impl*>(this)->find(id);
    }
};

Simulation::Simulation(NavMesh mesh) : m_impl(std::make_unique<Impl>(std::move(mesh))) {}

Simulation::~Simulation() = default;
Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

Result<AgentId> Simulation::add_agent(const AgentConfig& config) {
    // A moved-from simulation cannot accept new agents.
    if (!m_impl) {
        return std::unexpected(Error{ErrorCode::NotFound, "simulation is moved-from"});
    }
    // Validate all scalars before mutating any observable state (incl. next assignable id).
    if (!finite_vec(config.position)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "agent position is non-finite"});
    }
    if (!std::isfinite(config.radius) || !(config.radius > 0.0f)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "agent radius must be positive"});
    }
    if (!std::isfinite(config.max_speed) || !(config.max_speed > 0.0f)) {
        return std::unexpected(
            Error{ErrorCode::InvalidArgument, "agent max_speed must be positive"});
    }
    float eff_arrival{};
    if (const Result<void> ar = resolve_arrival(config.arrival_radius, config.radius, eff_arrival);
        !ar) {
        return std::unexpected(ar.error());
    }
    if (!m_impl->mesh.contains(config.position)) {
        return std::unexpected(
            Error{ErrorCode::OutsideMesh, "agent position is outside the navmesh"});
    }

    // Fully prepare the agent locally first: route to an optional goal WITHOUT consuming an
    // id or touching shared state, so a rejected add leaves everything untouched.
    Impl::LiveAgent agent{};
    agent.position = config.position;
    agent.velocity = {};
    agent.radius = config.radius;
    agent.max_speed = config.max_speed;
    agent.arrival_radius = eff_arrival;

    if (config.goal.has_value()) {
        if (!finite_vec(*config.goal)) {
            return std::unexpected(Error{ErrorCode::InvalidArgument, "goal point is non-finite"});
        }
        if (!m_impl->mesh.contains(*config.goal)) {
            return std::unexpected(
                Error{ErrorCode::OutsideMesh, "goal point is outside the navmesh"});
        }
        agent.goal = *config.goal;

        const Result<Path> route = find_path(m_impl->mesh, agent.position, *agent.goal);
        if (!route) {
            if (route.error().code != ErrorCode::NoPath) {
                return std::unexpected(route.error());
            }
            // Disconnected in-mesh goal is not an error here: succeed with NoPath status.
            agent.status = AgentStatus::NoPath;
            agent.waypoints.clear();
        } else {
            // Build waypoints (dropping the start-coincident vertex) and apply immediate-reach.
            agent.waypoints.clear();
            bool skipped_start = false;
            for (const Vec2 p : (*route).points) {
                if (!skipped_start && len_sq(p - agent.position) <= kEpsSq) {
                    skipped_start = true;
                    continue;
                }
                agent.waypoints.push_back(p);
            }
            if (agent.goal.has_value() && length(*agent.goal - agent.position) <= eff_arrival) {
                agent.velocity = {};
                agent.waypoints.clear();
                agent.status = AgentStatus::Reached;
            } else {
                agent.status = AgentStatus::Moving;
            }
        }
    }

    // Commit only after full validation succeeds.
    const std::uint32_t assigned_id = m_impl->next_id++;
    if (m_impl->next_id == 0) {
        m_impl->next_id = 1;
    }
    agent.id = assigned_id;
    m_impl->live.push_back(std::move(agent));
    return AgentId{assigned_id};
}

Result<void> Simulation::remove_agent(AgentId id) {
    if (!m_impl) {
        return std::unexpected(Error{ErrorCode::NotFound, "simulation is moved-from"});
    }
    auto& live = m_impl->live;
    for (auto it = live.begin(); it != live.end(); ++it) {
        if (it->id == id.value) {
            live.erase(it);
            return {};
        }
    }
    return std::unexpected(Error{ErrorCode::NotFound, "unknown agent id"});
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius) {
    if (!m_impl) {
        return std::unexpected(Error{ErrorCode::NotFound, "simulation is moved-from"});
    }
    Impl::LiveAgent* a = m_impl->find(id.value);
    if (!a) {
        return std::unexpected(Error{ErrorCode::NotFound, "unknown agent id"});
    }
    // Validate inputs without yet mutating committed state.
    if (!finite_vec(goal)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "goal point is non-finite"});
    }
    float eff_arrival{};
    if (const Result<void> ar = resolve_arrival(arrival_radius, a->radius, eff_arrival); !ar) {
        return std::unexpected(ar.error());
    }
    if (!m_impl->mesh.contains(goal)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "goal point is outside the navmesh"});
    }

    const Result<Path> route = find_path(m_impl->mesh, a->position, goal);
    if (!route && route.error().code != ErrorCode::NoPath) {
        return std::unexpected(route.error());
    }

    // Commit validated values transactionally.
    a->arrival_radius = eff_arrival;
    a->goal = goal;
    a->velocity = {};

    if (!route) {
        // Disconnected in-mesh goal: terminal NoPath with zero velocity.
        a->status = AgentStatus::NoPath;
        a->waypoints.clear();
        return {};
    }

    // Build waypoints (dropping the start-coincident vertex) and apply immediate-reach.
    a->waypoints.clear();
    bool skipped_start = false;
    for (const Vec2 p : (*route).points) {
        if (!skipped_start && len_sq(p - a->position) <= kEpsSq) {
            skipped_start = true;
            continue;
        }
        a->waypoints.push_back(p);
    }
    if (a->goal.has_value() && length(*a->goal - a->position) <= eff_arrival) {
        a->velocity = {};
        a->waypoints.clear();
        a->status = AgentStatus::Reached;
    } else if (a->status != AgentStatus::NoPath) {
        a->status = AgentStatus::Moving;
    }
    return {};
}

Result<void> Simulation::clear_goal(AgentId id) {
    if (!m_impl) {
        return std::unexpected(Error{ErrorCode::NotFound, "simulation is moved-from"});
    }
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

// --- Per-substep steering -------------------------------------------------------

Result<void> Simulation::step(float seconds) {
    if (!std::isfinite(seconds) || !(seconds >= 0.0f)) {
        return std::unexpected(
            Error{ErrorCode::InvalidArgument, "step duration must be finite and non-negative"});
    }
    if (seconds == 0.0f || !m_impl) {
        return m_impl
                   ? Result<void>{}
                   : std::unexpected(Error{ErrorCode::InvalidArgument, "simulation is moved-from"});
    }

    auto& live = m_impl->live;
    if (live.empty()) {
        return {};
    }

    const NavMesh& mesh = m_impl->mesh;
    float remaining = seconds;
    int steps = 0;
    while (remaining > 0.0f) {
        const float sub_dt = std::min(kMaxSubStep, remaining);
        const std::size_t n = live.size();

        // Snapshot desired velocities toward each moving agent's current target first so all
        // pairwise avoidance decisions use an immutable configuration before committing.
        std::vector<Vec2> vel(n);
        for (std::size_t i = 0; i < n; ++i) {
            if (live[i].status != AgentStatus::Moving) {
                vel[i] = {};
                continue;
            }
            const Vec2 tgt = (!live[i].waypoints.empty()) ? live[i].waypoints.front()
                                                          : live[i].goal.value_or(live[i].position);
            vel[i] = desired_unit(tgt - live[i].position) * live[i].max_speed;
        }

        // Reciprocal velocity-obstacle selection over the substep horizon using snapshot
        // positions. gap s = |delta| - rsum already encodes separation; the one-substep
        // non-overlap bound derives directly from that gap: post-step gap stays >= 0 means
        // closing' <= gap/dt. Coincident centres fall back to a stable ID-derived axis.
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                const Vec2 delta = live[j].position - live[i].position;
                const float rsum = live[i].radius + live[j].radius;
                const float dist = std::sqrt(len_sq(delta));
                const Vec2 axis = (dist > kEpsilon)
                                      ? delta * (1.0f / dist)
                                      : escape_dir(static_cast<std::uint32_t>(live[i].id) ^
                                                   (static_cast<std::uint32_t>(live[j].id) << 1));
                const float gap = dist - rsum; // positive => separated, negative => overlapped
                const Vec2 rel = vel[i] - vel[j];
                const float closing = dot(rel, axis);
                const float allowed_closing = gap / sub_dt;
                if (closing > allowed_closing) {
                    const float excess = closing - allowed_closing;
                    const Vec2 adj = axis * (excess * 0.5f);
                    vel[i] = vel[i] - adj;
                    vel[j] = vel[j] + adj;
                }
                // Preserve the speed-budget invariant after adjustment.
                const float si = length(vel[i]);
                if (si > live[i].max_speed && si > 0.0f) {
                    vel[i] = vel[i] * (live[i].max_speed / si);
                }
                const float sj = length(vel[j]);
                if (sj > live[j].max_speed && sj > 0.0f) {
                    vel[j] = vel[j] * (live[j].max_speed / sj);
                }
            }
        }

        // Commit all positions together with clamped displacement plus containment projection.
        bool any_moving = false;
        for (std::size_t i = 0; i < n; ++i) {
            Impl::LiveAgent& a = live[i];
            if (a.status != AgentStatus::Moving) {
                a.velocity = {};
                continue;
            }
            any_moving = true;
            const Vec2 prev_pos = a.position;
            const Vec2 raw_next = prev_pos + vel[i] * sub_dt;
            const float move_budget = a.max_speed * sub_dt;
            Vec2 next_pos = raw_next;
            const float disp_len = length(raw_next - prev_pos);
            if (disp_len > move_budget && disp_len > 0.0f) {
                next_pos = prev_pos + (raw_next - prev_pos) * (move_budget / disp_len);
            }
            a.position = project_inside(mesh, prev_pos, next_pos);
            a.velocity = (sub_dt > 0.0f) ? (a.position - prev_pos) * (1.0f / sub_dt) : Vec2{};

            const Vec2 tgt =
                (!a.waypoints.empty()) ? a.waypoints.front() : a.goal.value_or(a.position);
            if (length(tgt - a.position) <= a.arrival_radius) {
                if (!a.waypoints.empty()) {
                    a.waypoints.erase(a.waypoints.begin());
                } else if (a.goal.has_value()) {
                    // Within arrival radius of the goal: stop here without teleporting. The
                    // goal value is retained until clear_goal resets it.
                    a.velocity = {};
                    a.status = AgentStatus::Reached;
                }
            }
        }

        remaining -= sub_dt;
        ++steps;
        // Once no agent can still make progress and elapsed time has been consumed in whole
        // substeps, further simulation changes nothing observable; stop to guarantee termination.
        if (!any_moving || steps >= kMaxSubsteps) {
            break;
        }
    }

    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept {
    if (!m_impl) {
        return std::nullopt;
    }
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

std::size_t Simulation::agent_count() const noexcept {
    return m_impl ? m_impl->live.size() : static_cast<std::size_t>(0);
}

} // namespace vwmini
