#include <vwmini/simulation.hpp>

#include <vwmini/detail.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <tuple>
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
    static Result<void> assign_goal(Impl& impl, Agent& a, Vec2 goal_pos, float arrival_radius);

    explicit Impl(NavMesh mesh_) : mesh(std::move(mesh_)) {}
};

Simulation::Simulation(NavMesh mesh) : m_impl(std::make_unique<Impl>(std::move(mesh))) {}

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

// 90-degree rotations: deterministic tangential directions for sidestepping.
Vec2 rotate90_ccw(Vec2 v) noexcept
{
    return {-v.y, v.x};
}
Vec2 rotate90_cw(Vec2 v) noexcept
{
    return {v.y, -v.x};
}

// Scale v back to at most max_speed (preserves direction; 0 stays 0).
Vec2 cap_speed(Vec2 v, float max_speed) noexcept
{
    const float L = length(v);
    if (L > max_speed) {
        v = v * (max_speed / L);
    }
    return v;
}

// Minimum centre-to-centre distance of two discs moving on straight lines over
// [0, sub_dt]. r0 = pos_i - pos_k, rd = (v_i - v_k) * sub_dt is the relative
// displacement over the interval; the closest separation of the segment
// r0 + t*rd (t in [0,1]) is returned.
float swept_min_sep(Vec2 r0, Vec2 rd) noexcept
{
    const float rr = dot(rd, rd);
    float t = 0.5f; // rd == 0 -> constant separation
    if (rr > 0.0f) {
        t = -dot(r0, rd) / rr;
        if (t < 0.0f) {
            t = 0.0f;
        } else if (t > 1.0f) {
            t = 1.0f;
        }
    }
    return length(r0 + rd * t);
}

} // namespace

// Apply an in-mesh goal: compute route and transition state (SIM-007).
Result<void> Simulation::Impl::assign_goal(Impl& impl, Agent& a, Vec2 goal_pos,
                                           float arrival_radius)
{
    float eff = 0.0f;
    if (!resolve_arrival(arrival_radius, a.radius, eff)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "invalid arrival radius"});
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

    if (a.route.size() < 2 || length(a.position - goal_pos) <= a.effective_arrival_radius) {
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
        return std::unexpected(Error{ErrorCode::InvalidArgument, "invalid agent configuration"});
    }
    if (!m_impl->mesh.contains(config.position)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "agent position is outside the mesh"});
    }
    // Validate the optional goal before mutating any state so a rejected add
    // leaves agents untouched (ids stay durable, agent_count unchanged).
    if (config.goal.has_value() && !m_impl->mesh.contains(config.goal.value())) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "agent goal is outside the mesh"});
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
        Result<void> r = Impl::assign_goal(*m_impl, a, g, config.arrival_radius);
        if (!r) {
            m_impl->agents.pop_back(); // rollback the in-mesh mutation
            return std::unexpected(r.error());
        }
    }
    return AgentId{id};
}

Result<void> Simulation::remove_agent(AgentId id)
{
    if (id.value == 0 || id.value > m_impl->agents.size() || !m_impl->agents[id.value - 1].alive) {
        return std::unexpected(Error{ErrorCode::NotFound, "unknown or removed agent id"});
    }
    m_impl->agents[id.value - 1].alive = false;
    return {};
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius)
{
    if (id.value == 0 || id.value > m_impl->agents.size() || !m_impl->agents[id.value - 1].alive) {
        return std::unexpected(Error{ErrorCode::NotFound, "unknown or removed agent id"});
    }
    if (!detail::finite(goal)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite goal"});
    }
    float tmp = 0.0f;
    if (!resolve_arrival(arrival_radius, m_impl->agents[id.value - 1].radius, tmp)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "invalid arrival radius"});
    }
    auto& a = m_impl->agents[id.value - 1];
    if (!m_impl->mesh.contains(goal)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "goal is outside the mesh"});
    }
    return Impl::assign_goal(*m_impl, a, goal, arrival_radius);
}

Result<void> Simulation::clear_goal(AgentId id)
{
    if (id.value == 0 || id.value > m_impl->agents.size() || !m_impl->agents[id.value - 1].alive) {
        return std::unexpected(Error{ErrorCode::NotFound, "unknown or removed agent id"});
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
        return std::unexpected(Error{ErrorCode::InvalidArgument, "invalid step duration"});
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
    // Bound the substep *count* (not the simulated time). A finite but enormous
    // duration (e.g. FLT_MAX) must still advance the full 'seconds' of simulated
    // time; seconds/target_substep can otherwise overflow float and turn the
    // ceil -> size_t cast into undefined behaviour. We cap the count and set
    // sub_dt = seconds / n_sub, so total elapsed time is always exactly 'seconds'
    // (coarsening only for the extreme case). For any in-mesh goal the arrival
    // ramp (dist / sub_dt) still lands the agent on its goal within a couple of
    // substeps even when sub_dt is huge.
    constexpr std::size_t max_substeps = 100000;
    const float ratio = seconds / target_substep; // finite: seconds finite & > 0
    const std::size_t n_sub =
        (ratio > static_cast<float>(max_substeps))
            ? max_substeps
            : std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(ratio)));
    const float sub_dt = seconds / static_cast<float>(n_sub);

    struct Snap {
        Vec2 pos;
        Vec2 vel;
        Vec2 dest;
        Vec2 goal_dir;   // unit vector toward dest (precomputed so steps 2 / 3 share it)
        float goal_dist; // distance to dest
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
            const Vec2 to_dest = dest - a->position;
            snap.push_back({a->position, a->velocity, dest, normalized(to_dest), length(to_dest),
                            a->radius, a->max_speed, a->status == AgentStatus::Moving});
        }

        // Cap a velocity to max_speed, clamp it to stay in-mesh, and push it as a
        // candidate for agent i.  Duplicates of the desired velocity c0
        // (cands[i][0]) are skipped so the candidate set stays bounded.
        std::vector<Vec2> des(snap.size(), Vec2{0.0f, 0.0f}); // desired velocity per agent
        std::vector<Vec2> v(snap.size(), Vec2{0.0f, 0.0f});
        std::vector<std::vector<Vec2>> cands(snap.size());
        const auto add_cand = [&](std::size_t i, Vec2 c) {
            c = cap_speed(c, snap[i].max_speed);
            if (!cands[i].empty() && length(c - cands[i][0]) < 1e-5f) {
                return; // identical to desired: redundant
            }
            const float scale = clamp_into_mesh(mesh, snap[i].pos, c, sub_dt);
            cands[i].push_back(c * scale);
        };

        // 2. Build a bounded, deterministic candidate velocity set per moving
        //    agent, clamped into the mesh up-front (via add_cand) so every
        //    selection is containment-safe by construction:
        //      c0 desired (arrival-capped)
        //      c1 yield (half-speed)
        //      c2 stop
        //      c3/c4 lateral +/-90 deg of desired (strafe)
        //      c5/c6 forward+/-lateral (crossing / overtaking / prefix pass)
        for (std::size_t i = 0; i < snap.size(); ++i) {
            if (!snap[i].moving) {
                continue;
            }
            const float dist = snap[i].goal_dist;
            const Vec2 desired = (dist > 0.0f)
                                     ? snap[i].goal_dir * std::min(snap[i].max_speed, dist / sub_dt)
                                     : Vec2{0.0f, 0.0f};
            des[i] = desired;
            const Vec2 lat_l = rotate90_ccw(desired);
            const Vec2 lat_r = rotate90_cw(desired);
            add_cand(i, desired);                       // c0: desired
            add_cand(i, desired * 0.5f);                // c1: yield (half-speed)
            add_cand(i, Vec2{0.0f, 0.0f});              // c2: stop
            add_cand(i, lat_l);                         // c3: strafe CCW
            add_cand(i, lat_r);                         // c4: strafe CW
            add_cand(i, desired * 0.7f + lat_l * 0.7f); // c5: forward+lat CCW
            add_cand(i, desired * 0.7f + lat_r * 0.7f); // c6: forward+lat CW
            v[i] = cands[i][0];
        }

        // 2b. Reciprocal separating / tangential-sidestep candidates per live
        //     neighbour, but only for proximate pairs so the candidate set stays
        //     bounded.  These let two agents *cross* (tangential slide along the
        //     contact) or *resolve* an overlap (push along the contact normal).
        //     Each candidate is capped and containment-shielded by add_cand
        //     (which also drops duplicates of c0).
        const float contact_bias = 1e-3f;
        const float prox_margin = 0.5f; // generate sidestep options within ~rsum+margin
        for (std::size_t i = 0; i < snap.size(); ++i) {
            if (!snap[i].moving) {
                continue;
            }
            for (std::size_t k = 0; k < snap.size(); ++k) {
                if (k == i) {
                    continue;
                }
                const Vec2 diff = snap[i].pos - snap[k].pos;
                const float dist = length(diff);
                const float rsum = snap[i].radius + snap[k].radius;
                if (dist >= rsum + prox_margin) {
                    continue; // too far: neighbours cannot collide this substep
                }
                const Vec2 n_hat = (dist > 0.0f) ? diff * (1.0f / dist) : Vec2{1.0f, 0.0f};
                const float rel_n = dot(des[i] - des[k], n_hat); // <0 => i is approaching k
                const Vec2 t1 = rotate90_ccw(n_hat);
                const Vec2 t2 = rotate90_cw(n_hat);
                // Hard contact-resolution push along the normal (only when
                // overlapping so approaching agents are nudged apart).
                if (dist < rsum) {
                    const float needed = (rsum - dist) / sub_dt + contact_bias;
                    add_cand(i, des[i] - n_hat * (0.5f * needed));
                }
                // Full separation + both tangential sidesteps.
                add_cand(i, n_hat * snap[i].max_speed);
                add_cand(i, t1 * snap[i].max_speed);
                add_cand(i, t2 * snap[i].max_speed);
                // Reciprocal yield: give way along the contact normal when approaching.
                if (rel_n < 0.0f) {
                    add_cand(i, des[i] - n_hat * (0.5f * rel_n));
                    add_cand(i, des[i] - n_hat * rel_n);
                }
            }
        }

        // 3. Candidate-selection avoidance: each moving agent best-responds to its
        //    neighbours' current choices.  Evaluation is index-ordered
        //    (Gauss-Seidel): an earlier agent commits first, which breaks the
        //    coincident symmetry a purely simultaneous sweep cannot.  A choice is
        //    ranked so goal-directed motion wins, ties resolve by a stable index,
        //    and the chosen velocities are committed together once the selection
        //    has settled.
        struct Score {
            bool feasible;
            float progress;
            float overlap;
            float separation;
            std::size_t idx;
        };
        // Rank key: lower = preferred (collision-free first, least overlap, most
        // progress toward the waypoint, most separation, then stable index).
        const auto rank = [](const Score& s) {
            return std::make_tuple(!s.feasible, s.overlap, -s.progress, -s.separation, s.idx);
        };
        std::vector<std::size_t> pick(snap.size(), 0);
        for (int round = 0; round < 8; ++round) {
            bool changed = false;
            for (std::size_t i = 0; i < snap.size(); ++i) {
                if (cands[i].empty()) {
                    continue; // non-moving: a stationary obstacle
                }
                const Vec2 gdir = snap[i].goal_dir;
                Score best = {false,
                              std::numeric_limits<float>::lowest(), // progress: min = worst
                              std::numeric_limits<float>::max(),    // overlap:   max = worst
                              std::numeric_limits<float>::lowest(), // separation: min = worst
                              pick[i]};
                for (std::size_t ci = 0; ci < cands[i].size(); ++ci) {
                    const Vec2 cv = cands[i][ci];
                    float progress = dot(cv, gdir);
                    float overlap = 0.0f;
                    float separation = 0.0f;
                    bool feasible = true;
                    for (std::size_t j = 0; j < snap.size(); ++j) {
                        if (j == i) {
                            continue;
                        }
                        const float rsum = snap[i].radius + snap[j].radius;
                        const Vec2 r0 = snap[i].pos - snap[j].pos;
                        const Vec2 rd = (cv - v[j]) * sub_dt;
                        const float s = swept_min_sep(r0, rd);
                        separation += s;
                        if (s < rsum) {
                            feasible = false;
                            overlap += (rsum - s);
                        }
                    }
                    const Score sc = {feasible, progress, overlap, separation, ci};
                    if (rank(sc) < rank(best)) {
                        best = sc;
                    }
                }
                if (best.idx != pick[i]) {
                    pick[i] = best.idx;
                    v[i] = cands[i][best.idx];
                    changed = true;
                }
            }
            if (!changed) {
                break;
            }
        }

        // 4. Apply motion with waypoint-overshoot protection, then update state.
        //    Candidates were clamped into the mesh during generation, so no
        //    separate containment pass is needed; the clamp below only guards
        //    against waypoint-overshoot or projection drift pushing an agent
        //    across a mesh boundary.
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

            const float cscale = clamp_into_mesh(mesh, a->position, delta, 1.0f);
            a->velocity = (sub_dt > 0.0f) ? delta * (1.0f / sub_dt) : Vec2{0.0f, 0.0f};
            a->position = a->position + delta * cscale;

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
                if (length(a->position - a->goal.value()) <= a->effective_arrival_radius) {
                    a->status = AgentStatus::Reached;
                    a->velocity = Vec2{0.0f, 0.0f};
                }
            }
        }

        // 5. Early termination: once no live agent is still moving, the remaining
        //    duration has no further effect on state, so stop short of the bounded
        //    substep count (e.g. FLT_MAX reaches its goal in a couple of
        //    substeps).
        bool any_moving = false;
        for (auto* a : live) {
            if (a->status == AgentStatus::Moving) {
                any_moving = true;
                break;
            }
        }
        if (!any_moving) {
            break;
        }
    }

    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept
{
    if (id.value == 0 || id.value > m_impl->agents.size() || !m_impl->agents[id.value - 1].alive) {
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
