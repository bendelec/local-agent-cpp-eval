// Simulation core (SIM-005..SIM-013), waypoint motion (S6.1) and local disc
// avoidance (S6.2). Single-threaded; deterministic float math; PIMPL per the
// public header.
#include <vwmini/simulation.hpp>
#include "simulation_impl.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace vwmini {

namespace {

// -1.0f is the arrival-radius sentinel ("use the agent radius"); every other
// negative value is invalid (SIM-006). -0.0f is zero and valid.
constexpr float kArrivalSentinel = -1.0f;

[[nodiscard]] bool valid_arrival(float arrival_radius)
{
    return arrival_radius >= 0.0f || arrival_radius == kArrivalSentinel;
}

[[nodiscard]] float effective_arrival(float arrival_radius, float radius)
{
    return arrival_radius == kArrivalSentinel ? radius : arrival_radius;
}

// Non-const lookup of a live agent slot (slot i == AgentId{i+1}); the const
// overload is the single bounds/alive check, used by both mutation and query.
[[nodiscard]] const SimAgent* live_agent(const std::vector<SimAgent>& agents,
                                         AgentId id)
{
    if (id.value == 0)
        return nullptr;
    const std::size_t slot = static_cast<std::size_t>(id.value) - 1;
    if (slot >= agents.size() || !agents[slot].alive)
        return nullptr;
    return &agents[slot];
}
[[nodiscard]] SimAgent* live_agent(std::vector<SimAgent>& agents, AgentId id)
{
    return const_cast<SimAgent*>(
        live_agent(static_cast<const std::vector<SimAgent>&>(agents), id));
}

// Clears motion state (velocity, route, route index); used when an agent
// enters a terminal or idle state (apply_goal, clear_goal).
void reset_motion(SimAgent& agent)
{
    agent.velocity = {};
    agent.route.clear();
    agent.route_index = 0;
}

[[nodiscard]] Result<void> not_found_error()
{
    return std::unexpected(Error{ErrorCode::NotFound, "agent id is unknown or removed"});
}

[[nodiscard]] Error invalid_argument(std::string message)
{
    return Error{ErrorCode::InvalidArgument, std::move(message)};
}

[[nodiscard]] Error outside_mesh_error()
{
    return Error{ErrorCode::OutsideMesh, "point is not contained by the mesh"};
}

// 48 bisection iterations resolve t to ~2^-48, far below float epsilon.
constexpr int kClampBisections = 48;
constexpr int kMaxSubsteps = 1000000;

// ---- Local disc avoidance (SIM-010…SIM-012) -----------------------------
// All constants below are the tuned values from WP6 (S6.2). They were chosen
// so the deterministic SIM-011 acceptance scenarios (crossing, overtaking,
// angled crossing) keep pair distances >= r_i + r_j - 1e-3 m while both
// agents reach their goals, and so initially overlapping discs separate.

// Prediction horizon (metres) for the closest-approach test: pairs whose
// predicted closest approach within this *distance* trigger a velocity-space
// correction. 1.5 m gives ~15 substeps of intent at the 1.0–1.4 m/s speeds
// used by the SIM-011 acceptance scenarios.
constexpr float kAvoidLookahead = 1.5f;

// Extra clearance (metres) beyond the touching distance r_i + r_j used by
// the closest-approach test. 0.4 m triggers the correction early enough for
// the crossing scenario to clear the intersection without overlap.
constexpr float kAvoidMargin = 0.4f;

// Weight (in [0,1]) blending the waypoint-steering velocity toward the
// conflict-free avoidance velocity in the closest-approach branch. 1.0 takes
// the avoidance velocity fully; 0.0 keeps pure waypoint steering.
constexpr float kAvoidWeight = 1.0f;

// Extra fraction added on top of the overlap-depth term in the
// already-overlapping branch, pushing apart overlapping discs actively
// (not just stopping the approach). The push must STRICTLY dominate the
// worst-case mutual approach (ms_a + ms_b), which is what happens when
// both agents steer directly at each other at full speed while their
// discs overlap. With boost = 2.0 the relative separation speed while
// overlapping is S = (ms_a + ms_b) * (1 + (r - d)/r) >= ms_a + ms_b > 0
// for ANY overlap depth -- the pair separates past contact in finite
// time. Any boost <= 1 only cancels the mutual approach (boost = 1
// leaves S = (ms_a + ms_b) * (r - d)/r, which vanishes at contact and
// makes a head-on pair converge asymptotically to permanent contact,
// still reporting Moving and never reaching its goal).
constexpr float kAvoidSeparationBoost = 2.0f;

// Deterministic, anti-symmetric fallback direction when two overlapping
// agents coincide exactly (dist == 0): the smaller slot id pushes toward
// +x, the larger toward -x.
[[nodiscard]] constexpr Vec2 fallback_dir(std::size_t i, std::size_t j) noexcept
{
    return i < j ? Vec2{1.0f, 0.0f} : Vec2{-1.0f, 0.0f};
}

// Fraction of max_speed at which a near-zone agent drifts away from the
// other agent. The velocity-obstacle cone (clearance radius R) is degenerate
// for dist <= R: the half-angle asin(R / dist) reaches 90 deg and the
// boundary projection is ill-conditioned (its direction is dominated by a
// near-zero perpendicular component whose sign flips with sub-millimetre
// position changes). In that zone the pair slides around instead: the
// approaching component of the velocity is dropped, the lateral component
// is kept, and the agent gains the outward drift. While both agents of a
// conflicting pair are in the zone the centre-line distance is strictly
// increasing (>= 2*drift*max_speed for a head-on pair, >= drift*max_speed
// one-sided), so the pair can never re-enter overlap from the zone and it
// leaves the zone in bounded time. An agent that is landing on its own
// target (within max_speed*dt of it) is exempt: it glides to a stop, the
// relative approach decays, and the overlap push (dist < r) still guards
// physical contact -- without the exemption an agent whose goal lies inside
// the zone of a parked agent could never land it (SIM-011/SIM-013).
constexpr float kAvoidSlideDrift = 0.25f;

// Computes the local avoidance velocity for one agent from the substep
// snapshot (SIM-010: reads only snapshot positions and the pre-avoidance
// steer velocities of all agents, never state mutated earlier in this
// substep). `self_k` is the agent's index in the snapshot arrays. The
// result is a velocity of magnitude <= max_speed that the caller stores as
// the decided substep velocity.
//
// For each live other agent the function considers two cases:
//   1. Already overlapping (dist < r): the agent is pushed apart along the
//      centre line, with a deterministic anti-symmetric fallback direction
//      when centres coincide exactly (SIM-010 edge case).
//   2. Predicted closest approach inside the horizon: for dist >= R the
//      agent's velocity is projected onto the nearest point outside the RVO
//      velocity obstacle (the double cone in relative-velocity space with
//      apex at -rel/t and half-angle asin(R / dist)), yielding the
//      conflict-free velocity; for r <= dist < R (degenerate zone: the cone
//      half-angle reaches 90 deg and the projection is ill-conditioned) the
//      agent slides around the other one instead (approach component
//      dropped, lateral kept, small outward drift -- see kAvoidSlideDrift),
//      unless it is landing on its own target, in which case it glides to
//      the stop uncorrected (the overlap push still guards contact).
//      Blended toward the original steer with kAvoidWeight.
// The final velocity is clamped to max_speed (SIM-012).
[[nodiscard]] Vec2 avoidance_velocity(const SimAgent& self, std::size_t self_k,
                                      const std::vector<SimAgent>& agents,
                                      const std::vector<std::size_t>& live_slots,
                                      const std::vector<Vec2>& snap_pos,
                                      const std::vector<Vec2>& steer,
                                      bool is_landing)
{
    const std::size_t self_slot = live_slots[self_k];
    const float r_i = self.radius;
    const float max_speed = self.max_speed;
    Vec2 v = steer[self_k];
    for (std::size_t j_k = 0; j_k < live_slots.size(); ++j_k)
    {
        if (j_k == self_k)
            continue;
        const std::size_t j_slot = live_slots[j_k];
        const SimAgent& other = agents[j_slot];
        const float r = r_i + other.radius;
        const Vec2 rel = snap_pos[j_k] - snap_pos[self_k];
        const float dist = length(rel);
        if (dist < r)
        {
            // 1) Already overlapping: push apart along the centre line.
            //    `rel` points from self toward the other agent, so the
            //    separation direction is -u (push AWAY from the other
            //    agent). The push magnitude is proportional to the overlap
            //    depth plus a fixed boost (see kAvoidSeparationBoost) so
            //    the discs actively separate rather than merely stop
            //    approaching. Deterministic anti-symmetric fallback when
            //    the centres coincide exactly (SIM-010 edge case): the
            //    smaller slot id pushes toward +x, the larger toward -x.
            const float magnitude =
                max_speed * ((r - dist) / r + kAvoidSeparationBoost);
            if (dist > 0.0f)
                v = v - normalized(rel) * magnitude;
            else
                v = v + fallback_dir(self_slot, j_slot) * magnitude;
        }
        else
        {
            // 2) Predicted closest approach (RVO velocity-obstacle test).
            //    t is the time of closest approach; if it falls inside the
            //    horizon (converted to a distance via the relative speed)
            //    and the closest-approach distance is inside the clearance
            //    radius, the desired velocity is projected to the
            //    conflict-free half-space.
            const Vec2 relv = steer[j_k] - v;
            const float q = dot(rel, relv);
            if (q >= 0.0f)
                continue;  // separating: no conflict.
            const float rv2 = dot(relv, relv);
            const float t = -q / (rv2 > 1e-12f ? rv2 : 1e-12f);
            const float rel_speed = std::sqrt(rv2);
            if (rel_speed * t > kAvoidLookahead)
                continue;  // closest approach beyond the horizon.
            const Vec2 cpa = rel + relv * t;
            const float dca = length(cpa);
            const float R = r + kAvoidMargin;
            if (dca >= R)
                continue;  // clearance already sufficient.
            const Vec2 u = rel * (1.0f / dist);  // unit rel
            Vec2 v_new;
            if (dist < R)
            {
                // Degenerate zone (see kAvoidSlideDrift): the cone
                // half-angle is >= 90 deg and its boundary projection is
                // ill-conditioned. Slide around the other agent: drop the
                // approaching component of the velocity (if any), keep the
                // lateral component, gain the outward drift. A receding
                // agent is left alone: the approaching partner of the pair
                // slides, which already makes the centre-line distance grow.
                // A landing agent glides to its own target uncorrected (see
                // kAvoidSlideDrift): its velocity is the small to/dt gliding
                // speed, it stops at the target, and the overlap push below
                // (dist < r) still guards physical contact.
                const float along = dot(v, u);
                if (!is_landing && along > 0.0f)
                    v_new = v - u * (along + kAvoidSlideDrift * max_speed);
                else
                    v_new = v;
            }
            else
            {
                // RVO velocity-obstacle projection (Smit et al. 2005): the
                // forbidden set in relative-velocity space is the double cone
                // with apex at -rel/t and half-angle asin(R / dist). Project
                // the current relative velocity w onto the nearest point
                // outside the cone to obtain the new velocity.
                const Vec2 w = v - steer[j_k];  // current rel vel
                // a = w - (-rel/t): vector from cone apex to w.
                const Vec2 a = w + rel * (1.0f / t);
                // sin_h = R / dist <= 1 (guaranteed by dist >= R); clamp
                // defensively against float rounding.
                const float sin_h = R / dist < 1.0f ? R / dist : 1.0f;
                const float cos_h = std::sqrt(1.0f - sin_h * sin_h);
                // Decompose a into axial (along u) and perpendicular parts.
                const float a_along = dot(a, u);
                const Vec2 a_perp = a - u * a_along;
                const float d = length(a_perp);
                // Cone boundary half-width at axial distance a_along:
                //   w_boundary = |a_along| * tan(half-angle)
                const float w_boundary =
                    cos_h > 1e-12f ? std::fabs(a_along) * sin_h / cos_h : 0.0f;
                if (d >= w_boundary)
                {
                    // w is outside the cone: keep current velocity.
                    v_new = v;
                }
                else
                {
                    // w is inside the cone: project to the nearest boundary.
                    // Direction: sign of a_perp (deterministic; left normal
                    // of u when a_perp is zero, i.e. w on the cone axis).
                    const Vec2 perp_dir =
                        d > 1e-12f ? a_perp * (1.0f / d) : Vec2{-u.y, u.x};
                    const Vec2 new_w =
                        rel * (-1.0f / t) + u * a_along + perp_dir * w_boundary;
                    v_new = steer[j_k] + new_w;
                }
            }
            // Blend toward the conflict-free velocity with kAvoidWeight.
            // NOTE: do not fold kAvoidWeight == 1.0f away; the blend keeps the
            // zero-sign behaviour of the current expression (bit-identical).
            v = v * (1.0f - kAvoidWeight) + v_new * kAvoidWeight;
        }
    }
    // Final max-speed clamp (SIM-012).
    const float speed = length(v);
    if (speed > max_speed)
        v = v * (max_speed / speed);
    return v;
}

} // namespace

void apply_goal(SimAgent& agent, Vec2 goal, float arrival_radius,
                const NavMesh& mesh)
{
    const float effective = effective_arrival(arrival_radius, agent.radius);
    agent.goal = goal;
    agent.arrival_radius = effective;

    if (length(agent.position - goal) <= effective)
    {
        // Already inside the effective arrival radius: terminal Reached.
        agent.status = AgentStatus::Reached;
        reset_motion(agent);
        return;
    }

    const auto path = find_path(mesh, agent.position, goal);
    if (!path)
    {
        // SIM-007: a disconnected in-mesh goal succeeds with NoPath.
        agent.status = AgentStatus::NoPath;
        reset_motion(agent);
        return;
    }

    agent.status = AgentStatus::Moving;
    agent.velocity = {};
    agent.route = path->points;
    // The first point is the position itself; drop it so the route holds
    // only real waypoints (the last is the goal).
    agent.route.erase(agent.route.begin());
    agent.route_index = 0;
}

Simulation::Simulation(NavMesh mesh) : m_impl(std::make_unique<Impl>(std::move(mesh))) {}
Simulation::~Simulation() = default;
Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

Result<AgentId> Simulation::add_agent(const AgentConfig& config)
{
    // Validation order per SIM-005/SIM-006; first failure wins.
    if (!detail::is_finite(config.position) || !std::isfinite(config.radius) ||
        !std::isfinite(config.max_speed) || !std::isfinite(config.arrival_radius) ||
        (config.goal && !detail::is_finite(*config.goal)))
        return std::unexpected(invalid_argument("agent configuration must be finite"));
    if (config.radius <= 0.0f || config.max_speed <= 0.0f)
        return std::unexpected(invalid_argument("radius and max_speed must be positive"));
    if (!valid_arrival(config.arrival_radius))
        return std::unexpected(
            invalid_argument("arrival_radius must be non-negative or the -1.0f sentinel"));
    if (!m_impl->mesh.contains(config.position))
        return std::unexpected(outside_mesh_error());
    if (config.goal && !m_impl->mesh.contains(*config.goal))
        return std::unexpected(outside_mesh_error());

    SimAgent agent;
    agent.alive = true;
    agent.position = config.position;
    agent.radius = config.radius;
    agent.max_speed = config.max_speed;

    // Transactional: commit the slot only after the goal transition succeeds.
    if (config.goal)
        apply_goal(agent, *config.goal, config.arrival_radius, m_impl->mesh);

    m_impl->agents.push_back(agent);
    const AgentId id{m_impl->next_id++};
    ++m_impl->live_count;
    return id;
}

Result<void> Simulation::remove_agent(AgentId id)
{
    SimAgent* agent = live_agent(m_impl->agents, id);
    if (!agent)
        return not_found_error();
    agent->alive = false;
    --m_impl->live_count;
    return {};
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius)
{
    SimAgent* agent = live_agent(m_impl->agents, id);
    if (!agent)
        return not_found_error();
    if (!detail::is_finite(goal))
        return std::unexpected(invalid_argument("goal must be finite"));
    if (!valid_arrival(arrival_radius))
        return std::unexpected(
            invalid_argument("arrival_radius must be non-negative or the -1.0f sentinel"));
    if (!m_impl->mesh.contains(goal))
        return std::unexpected(outside_mesh_error());
    apply_goal(*agent, goal, arrival_radius, m_impl->mesh);
    return {};
}

Result<void> Simulation::clear_goal(AgentId id)
{
    SimAgent* agent = live_agent(m_impl->agents, id);
    if (!agent)
        return not_found_error();
    agent->goal.reset();
    agent->status = AgentStatus::Idle;
    reset_motion(*agent);
    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept
{
    const SimAgent* a = live_agent(m_impl->agents, id);
    if (!a)
        return std::nullopt;
    return AgentState{a->position, a->velocity, a->radius, a->max_speed, a->goal,
                      a->status};
}

std::size_t Simulation::agent_count() const noexcept
{
    return m_impl->live_count;
}

Result<void> Simulation::step(float seconds)
{
    // SIM-008: transactional validation; no state change on failure.
    if (!std::isfinite(seconds) || seconds < 0.0f)
        return std::unexpected(invalid_argument("step duration must be finite and non-negative"));
    if (seconds == 0.0f)
        return {};

    // Deterministic substep scheme: n substeps of equal duration dt. The
    // cap comparison runs BEFORE llround: for a huge finite duration
    // seconds / 0.1 exceeds LLONG_MAX and std::llround itself would be
    // undefined (SIM-008: large finite durations stay safe).
    const double scaled = static_cast<double>(seconds) / 0.1;
    long long n = scaled >= static_cast<double>(kMaxSubsteps)
                      ? kMaxSubsteps
                      : std::llround(scaled);
    if (n < 1)
        n = 1;
    const float dt = seconds / static_cast<float>(n);

    for (long long s = 0; s < n; ++s)
    {
        // (a) Snapshot (SIM-010): positions of all live agents in slot
        // order; velocity decisions below read only this snapshot and agent
        // configuration, never state mutated earlier in this substep.
        std::vector<std::size_t> live_slots;
        std::vector<Vec2> snap_pos;
        live_slots.reserve(m_impl->live_count);
        for (std::size_t i = 0; i < m_impl->agents.size(); ++i)
        {
            if (!m_impl->agents[i].alive)
                continue;
            live_slots.push_back(i);
            snap_pos.push_back(m_impl->agents[i].position);
        }

        // (b) Decide each live agent's substep velocity from the snapshot.
        // When an agent is close enough to its target waypoint, it "lands"
        // directly on it (exact position, no float accumulation error);
        // avoidance (b2) may override the landing for that substep.
        const std::size_t n_live = live_slots.size();
        std::vector<Vec2> decided(n_live);
        std::vector<char> landing(n_live, 0);
        std::vector<Vec2> steer(n_live);  ///< Pre-avoidance waypoint velocity.
        for (std::size_t k = 0; k < n_live; ++k)
        {
            SimAgent& a = m_impl->agents[live_slots[k]];
            Vec2 v{};
            if (a.status == AgentStatus::Moving)
            {
                // Moving always owns a non-empty route with route_index in range
                // (post-move transitions keep the invariant; see (e)).
                const Vec2 target = a.route[a.route_index];
                const Vec2 to = target - snap_pos[k];
                const float dist = length(to);
                if (dist > 0.0f)
                {
                    if (dist <= a.max_speed * dt)
                    {
                        // Land exactly on the waypoint: set position directly
                        // in the integration step to avoid float accumulation.
                        landing[k] = 1;
                        v = to * (1.0f / dt);
                    }
                    else
                        v = normalized(to) * a.max_speed;
                    if (length(v) > a.max_speed)  // Safety clamp.
                        v = normalized(v) * a.max_speed;
                }
            }
            steer[k] = v;
            decided[k] = v;
        }

        // (b2) Local disc avoidance (SIM-010…SIM-012). Each MOVING agent's
        // velocity is replaced by the RVO conflict-free velocity computed
        // exclusively from the snapshot (snapshot positions and pre-avoidance
        // steer velocities of all others; never state mutated earlier in this
        // substep), then clamped to max_speed. Idle/NoPath/Reached agents
        // keep v = {0,0} (they act as obstacles for others via the snapshot
        // but do not steer themselves). A landing agent whose velocity is
        // altered by avoidance forfeits the exact landing for this substep
        // (it re-decides next substep); the clamp guarantees
        // |v| <= max_speed either way (SIM-012).
        for (std::size_t k = 0; k < n_live; ++k)
        {
            const SimAgent& a = m_impl->agents[live_slots[k]];
            if (a.status != AgentStatus::Moving)
                continue;
            const Vec2 v_new = avoidance_velocity(a, k, m_impl->agents, live_slots,
                                                  snap_pos, steer, landing[k] != 0);
            decided[k] = v_new;
            if (landing[k] && v_new != steer[k])
                landing[k] = 0;  // Avoidance altered velocity: re-decide landing.
        }

        // (c) Integrate all agents from the snapshot-derived decisions.
        for (std::size_t k = 0; k < n_live; ++k)
        {
            SimAgent& a = m_impl->agents[live_slots[k]];
            if (landing[k])
            {
                // Exact landing on the target waypoint.
                a.position = a.route[a.route_index];
            }
            else
            {
                a.position = snap_pos[k] + decided[k] * dt;
            }
            a.velocity = decided[k];
        }

        // (d) Containment clamp (SIM-012): the agent may not leave the mesh.
        // The substep displacement delta = decided * dt may overflow float
        // (a huge finite duration with a fast agent: |decided| * dt > FLT_MAX
        // is inf) or be finite but far outside the mesh; a fixed bisection on
        // the parameter t in [0, 1] would need ~120 iterations to resolve the
        // boundary of a mesh of O(1) extent (leaving the agent outside), and
        // with an infinite delta it degenerates to from + delta * 0 = NaN.
        // Instead, work in arc length along the motion direction: expand
        // exponentially to an upper bound that is definitely outside (bounded
        // by |delta| when it is finite), then bisect that interval. t = 0 is
        // contained (the previous position was), so the result is contained
        // and finite.
        for (std::size_t k = 0; k < n_live; ++k)
        {
            SimAgent& a = m_impl->agents[live_slots[k]];
            if (detail::is_finite(a.position) && m_impl->mesh.contains(a.position))
                continue;
            const Vec2 from = snap_pos[k];
            const float spd = length(decided[k]);
            if (spd <= 0.0f)
            {
                // No motion decided: stay at the (contained) snapshot.
                a.position = from;
                continue;
            }
            const Vec2 dir = decided[k] * (1.0f / spd);
            const float L = length(a.position - from);  // may be inf
            float d_lo = 0.0f;
            float d_hi = std::min(1.0f, L);
            for (;;)
            {
                const Vec2 p = from + dir * d_hi;
                if (!detail::is_finite(p) || !m_impl->mesh.contains(p))
                    break;
                d_lo = d_hi;
                if (d_hi >= L)
                    break;  // the endpoint itself is contained
                d_hi = std::min(L, 2.0f * d_hi);
            }
            if (std::isfinite(d_hi))
            {
                for (int iter = 0; iter < kClampBisections; ++iter)
                {
                    const float mid = 0.5f * (d_lo + d_hi);
                    if (mid <= d_lo || mid >= d_hi)
                        break;  // float resolution exhausted
                    const Vec2 p = from + dir * mid;
                    if (detail::is_finite(p) && m_impl->mesh.contains(p))
                        d_lo = mid;
                    else
                        d_hi = mid;
                }
            }
            a.position = from + dir * d_lo;
        }

        // (e) Post-move transitions.
        for (std::size_t k = 0; k < n_live; ++k)
        {
            SimAgent& a = m_impl->agents[live_slots[k]];
            bool reached =
                a.goal && length(*a.goal - a.position) <= a.arrival_radius;
            if (!reached && a.status == AgentStatus::Moving)
            {
                // A waypoint counts as passed only on an EXACT landing:
                // the agent stands on it, from where the path-validated
                // next segment is walkable. Proximity after a clamped move
                // is not enough: an agent approaching a reflex corner may
                // end a substep within the advance threshold of the
                // waypoint while still short of it (or parked in the
                // boundary band), and the segment from that position to
                // the next waypoint cuts non-walkable space. Consuming the
                // waypoint there stranded the agent at the corner
                // (clamped to ~no motion, still reporting Moving), and the
                // failure only appeared for step durations whose substep
                // length leaves the remainder in (ms*dt, 2*ms*dt) -- a
                // normal 30 Hz caller hit it. The agent lands on the
                // waypoint next substep (it is within max_speed*dt) and
                // advances then, restoring guaranteed progress (SIM-009).
                if (landing[k])
                    ++a.route_index;
                // Route exhausted: the agent landed on the last route
                // point, which is the goal by construction of apply_goal.
                reached = a.route_index >= a.route.size();
            }
            if (reached)
            {
                // Snap to the goal for an exact terminal position.
                a.position = *a.goal;
                a.status = AgentStatus::Reached;
                a.velocity = {};
            }
        }
    }
    return {};
}

} // namespace vwmini
