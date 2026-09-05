// VWmini agent simulation implementation (WP5/WP6): lifecycle + bounded-velocity
// local avoidance. Steering selects a feasible velocity set per substep honoring the
// speed budget, disc-disc separation horizon, and mesh-containment invariants.
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

namespace {

float len_sq(Vec2 v) noexcept {
    return dot(v, v);
}

bool finite_vec(Vec2 p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

// normalized() already returns {0,0} for zero-length input per geometry.hpp contract.
Vec2 desired_unit(Vec2 v) noexcept {
    return normalized(v);
}

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

// Project `candidate` back onto the mesh union by backtracking along the step that
// produced it from `previous`, using binary search on containment. Keeps agents inside
// without exposing internal triangle storage to Simulation.
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
    if (!finite_vec(config.position)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "agent position is non-finite"});
    }
    if (!(config.radius > 0.0f)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "agent radius must be positive"});
    }
    if (!(config.max_speed >= 0.0f)) {
        return std::unexpected(
            Error{ErrorCode::InvalidArgument, "agent max_speed must be non-negative"});
    }
    if (!m_impl->mesh.contains(config.position)) {
        return std::unexpected(
            Error{ErrorCode::OutsideMesh, "agent position is outside the navmesh"});
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

    const std::uint32_t assigned_id = agent.id;
    m_impl->live.push_back(std::move(agent));

    if (config.goal.has_value()) {
        const Result<void> g = set_goal(AgentId{assigned_id}, *config.goal, config.arrival_radius);
        if (!g && g.error().code != ErrorCode::NoPath) {
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

Result<void> Simulation::remove_agent(AgentId id) {
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
    if (!finite_vec(goal)) {
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

Result<void> Simulation::clear_goal(AgentId id) {
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
// Steering is performed inline within step(): helper functions would need access to the
// private Simulation::Impl, so keeping the logic in the member avoids extra friending.

Result<void> Simulation::step(float seconds) {
    if (!(seconds >= 0.0f) || !std::isfinite(seconds)) {
        return std::unexpected(
            Error{ErrorCode::InvalidArgument, "step duration must be finite and non-negative"});
    }
    if (seconds == 0.0f) {
        return {};
    }

    auto& live = m_impl->live;
    if (live.empty()) {
        return {};
    }

    const NavMesh& mesh = m_impl->mesh;
    float remaining = seconds;
    while (remaining > 0.0f) {
        const float sub_dt = std::min(kMaxSubStep, remaining);
        const std::size_t n = live.size();

        // 1. Resolve any current disc overlaps so avoidance starts from a clean state.
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                const Vec2 delta = live[j].position - live[i].position;
                const float rsum = live[i].radius + live[j].radius;
                const float d2 = len_sq(delta);
                if (d2 >= rsum * rsum || d2 <= kEpsSq) {
                    continue;
                }
                const Vec2 nrm = escape_dir(static_cast<std::uint32_t>(live[i].id) ^
                                            (static_cast<std::uint32_t>(live[j].id) << 1));
                const float dist = std::sqrt(d2);
                const float pen = rsum - dist;
                const Vec2 push = nrm * (pen * 0.5f);
                live[i].position = live[i].position - push;
                live[j].position = live[j].position + push;
            }
        }

        // 2. Build desired velocities toward each moving agent's target, clamped to budget.
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

        // 3. Reciprocal velocity-obstacle selection over the substep horizon. For every pair
        //    whose signed gap s = |delta| - (r_i+r_j) would close below zero within `dt`, reduce
        //    the closing component of rel_vel equally (momentum-neutral). Stationary agents act
        //    as static obstacles with zero velocity. Deterministic index order keeps results
        //    reproducible across runs/simulations.
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
                // Distance shrinks at rate `closing`. Require the post-substep gap to stay
                // no smaller than rsum (contact): gap - closing'*dt >= rsum  =>  closing' <=
                // (gap-rsum)/dt.
                const float allowed_closing = (gap - rsum) / sub_dt;
                if (closing > allowed_closing) {
                    const float excess = closing - allowed_closing;
                    const Vec2 adj = axis * (excess * 0.5f);
                    vel[i] = vel[i] - adj;
                    vel[j] = vel[j] + adj;
                }
                // Re-clamp speeds to preserve the speed-budget invariant after adjustment.
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

        // 4. Integrate positions with clamped displacement and mesh projection, then advance
        //    along the waypoint chain or declare arrival at the goal.
        for (std::size_t i = 0; i < n; ++i) {
            Impl::LiveAgent& a = live[i];
            if (a.status != AgentStatus::Moving) {
                a.velocity = {};
                continue;
            }
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
                    a.position = *a.goal;
                    a.velocity = {};
                    a.goal.reset();
                    a.waypoints.clear();
                    a.status = AgentStatus::Reached;
                }
            }
        }

        remaining -= sub_dt;
    }

    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept {
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
    return m_impl ? m_impl->live.size() : 0;
}

} // namespace vwmini
