#include <vwmini/simulation.hpp>

#include "internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace vwmini {

namespace {

constexpr float kSubstep = 0.1f;

// Non-private internal state (free struct in an anonymous namespace) wrapped by
// the private Simulation::Impl type, so free helper functions can operate on it.
struct SimData {
    struct Agent {
        std::uint32_t id;
        Vec2 pos;
        Vec2 vel;
        float radius;
        float max_speed;
        float arrivalRadius; // effective arrival radius
        std::optional<Vec2> goal;
        AgentStatus status;
        std::vector<Vec2> route; // path points; last is the goal
        std::size_t routeIndex;  // next waypoint to reach
    };

    NavMesh mesh;
    std::vector<Agent> agents;
    std::uint32_t nextId = 1;
};

} // anonymous namespace: internal types

// Private mesh representation (Simulation::Impl forward-declared in the header).
struct Simulation::Impl {
    SimData data;
};

namespace {

SimData::Agent* findAgent(SimData& impl, AgentId id)
{
    for (auto& a : impl.agents) {
        if (a.id == id.value) {
            return &a;
        }
    }
    return nullptr;
}

// Advance past waypoints already within epsilon of the current position
// (never the goal itself, which is handled by arrival). Returns the next index
// to steer toward; does not mutate the agent.
std::size_t advanceRouteIndex(const SimData::Agent& a)
{
    std::size_t idx = a.routeIndex;
    while (idx + 1 < a.route.size() && internal::distLe(a.pos, a.route[idx])) {
        ++idx;
    }
    return idx;
}

// SIM-006: only -1.0f is the negative sentinel; every other negative value is invalid.
bool validArrivalRadius(float r)
{
    return internal::finite(r) && !(r < 0.0f && r != -1.0f);
}

float effectiveArrival(float radius, float arrivalRadius)
{
    return arrivalRadius == -1.0f ? radius : arrivalRadius;
}

// Applies SIM-007 transitions to an agent that has just been given a goal.
void assignGoal(SimData& impl, SimData::Agent& a, Vec2 goal, float effArrival)
{
    a.goal = goal;
    a.arrivalRadius = effArrival;
    a.vel = {0.0f, 0.0f};
    a.route.clear();
    a.routeIndex = 0;

    if (length(goal - a.pos) <= effArrival) {
        a.status = AgentStatus::Reached; // already within the effective arrival radius
        return;
    }

    auto p = find_path(impl.mesh, a.pos, goal);
    if (!p.has_value()) {
        a.status = AgentStatus::NoPath; // in-mesh goal but disconnected
        return;
    }

    a.status = AgentStatus::Moving;
    a.route = std::move(p.value().points);
    // route[0] is the start point (already at); the next waypoint to reach is route[1].
    a.routeIndex = (a.route.size() <= 1) ? 0 : 1;
}

// Snapshot of all live positions/velocities/radii read at substep start (SIM-010).
struct Snapshot {
    Vec2 pos;
    Vec2 vel;
    float radius;
};

// Deterministic candidate rotation offsets (degrees). Once forward motion is
// blocked, large lateral swerves are preferred: they build enough perpendicular
// offset for head-on/crossing agents to slip past (small diagonals close the
// head-on gap too fast and deadlock). +offsets rotate counter-clockwise
// relative to each agent's own travel direction.
constexpr float kOffsets[] = {
    0.0f,    90.0f, -90.0f, 45.0f,  -45.0f,  60.0f,  -60.0f,  30.0f,  -30.0f,  75.0f,  -75.0f,  105.0f,
    -105.0f, 15.0f, -15.0f, 120.0f, -120.0f, 135.0f, -135.0f, 150.0f, -150.0f, 165.0f, -165.0f, 180.0f,
};

// Deceleration factors tried for each rotation, so agents can slow before passing.
constexpr float kSpeeds[] = {1.0f, 0.7f, 0.5f, 0.3f, 0.1f};

// Safety margin kept between disc surfaces during normal avoidance (SIM-011).
constexpr float kAvoidMargin = 0.05f;

// Progressive clearance margins tried when seeking a candidate: prefer the full
// safety margin, then relax it (down to touching) so agents can slip past when
// there is no room for a margin instead of stopping permanently.
constexpr float kMargins[] = {kAvoidMargin, 0.03f, 0.01f, 0.0f};

// Lookahead horizon for the reciprocal swept collision check. Checking over a
// few substeps (not just the current one) makes agents begin swerving while
// there is still room, so head-on/crossing agents can pass instead of stopping
// at the margin boundary.
constexpr float kHorizon = 0.5f;

Vec2 decideVelocity(const SimData& impl, const SimData::Agent& a, std::size_t myIndex,
                    const std::vector<Snapshot>& snap, float dt)
{
    if (a.status != AgentStatus::Moving) {
        return {0.0f, 0.0f};
    }
    if (length(*a.goal - a.pos) <= a.arrivalRadius) {
        return {0.0f, 0.0f}; // arrival handled on apply
    }

    // Advance past waypoints already within epsilon of the snapshot position
    // (never the goal itself, which is handled by arrival).
    const std::size_t idx = advanceRouteIndex(a);
    const Vec2 target = a.route[idx];

    Vec2 desired = target - a.pos;
    const float dist = length(desired);
    if (dist <= 0.0f) {
        return {0.0f, 0.0f};
    }
    const Vec2 dir = desired * (1.0f / dist);
    const float speed = std::min(a.max_speed, dist / dt);
    const Vec2 desiredVel = dir * speed;

    const float baseAngle = std::atan2(desiredVel.y, desiredVel.x);
    constexpr float kDeg2Rad = 0.017453292519943295f;

    // Candidate search for a given per-pair clearance margin. Uses a reciprocal
    // swept check: the candidate's swept segment (this substep) must stay
    // (ri + rj + margin) away from the other agent's swept segment (other at
    // snapshot velocity), accounting for simultaneous motion (SIM-011).
    auto findCandidate = [&](float margin) -> std::optional<Vec2> {
        for (const float off : kOffsets) {
            for (const float sf : kSpeeds) {
                const float ang = baseAngle + off * kDeg2Rad;
                Vec2 cand{std::cos(ang), std::sin(ang)};
                cand = cand * (speed * sf);

                // Prefer candidates that make non-negative progress toward the target
                // (sideways is allowed, backward is not).
                if (dot(cand, desiredVel) < 0.0f) {
                    continue;
                }
                // Candidate must keep the agent inside the mesh.
                const Vec2 newpos = a.pos + cand * dt;
                if (!impl.mesh.contains(newpos)) {
                    continue;
                }
                bool collide = false;
                const Vec2 myEnd = a.pos + cand * kHorizon;
                for (std::size_t j = 0; j < snap.size(); ++j) {
                    if (j == myIndex) {
                        continue;
                    }
                    const float th = a.radius + snap[j].radius + margin;
                    const Vec2 otherEnd = snap[j].pos + snap[j].vel * kHorizon;
                    if (internal::distSegSeg(a.pos, myEnd, snap[j].pos, otherEnd) < th) {
                        collide = true;
                        break;
                    }
                }
                if (!collide) {
                    return cand;
                }
            }
        }
        return std::nullopt;
    };

    // Prefer a candidate that keeps the full safety margin; relax the required
    // per-pair clearance progressively (down to touching) so that agents can
    // slip past each other when there is no room for a margin, instead of
    // stopping permanently in a deadlock. SIM-011 acceptance covers feasible
    // open-space interactions, so this lets crossing/overtaking agents pass.
    for (const float margin : kMargins) {
        if (auto c = findCandidate(margin)) {
            return *c;
        }
    }

    // No collision-free candidate even at touching distance: stop rather than
    // push into a neighbour (SIM-012 robustness; guarantees no overlap, never
    // teleports).
    return {0.0f, 0.0f};
}

// Search kOffsets x kSpeeds for a velocity for agent `hi` whose committed
// substep swept segment [pos, pos + cand*dt] clears every other agent's
// committed swept segment (built from the already-decided `chosen`), with the
// usual progress and mesh-containment constraints. Returns nullopt to stop.
std::optional<Vec2> findCommittedCandidate(const SimData& impl, const std::vector<Snapshot>& snap,
                                           const std::vector<Vec2>& chosen, std::size_t hi, float dt)
{
    const SimData::Agent& a = impl.agents[hi];
    const std::size_t idx = advanceRouteIndex(a);
    const Vec2 target = a.route[idx];
    const Vec2 desired = target - a.pos;
    const float dist = length(desired);
    if (dist <= 0.0f) {
        return Vec2{0.0f, 0.0f}; // already at the waypoint: no motion needed
    }
    const Vec2 dir = desired * (1.0f / dist);
    const float speed = std::min(a.max_speed, dist / dt);
    const Vec2 desiredVel = dir * speed;
    const float baseAngle = std::atan2(desiredVel.y, desiredVel.x);
    constexpr float kDeg2Rad = 0.017453292519943295f;

    auto clear = [&](float margin, Vec2 cand) {
        const Vec2 myEnd = a.pos + cand * dt;
        for (std::size_t j = 0; j < snap.size(); ++j) {
            if (j == hi) {
                continue;
            }
            const Vec2 o0 = snap[j].pos;
            const Vec2 o1 = o0 + chosen[j] * dt;
            if (internal::distSegSeg(a.pos, myEnd, o0, o1) < a.radius + snap[j].radius + margin) {
                return false;
            }
        }
        return true;
    };

    for (const float margin : kMargins) {
        for (const float off : kOffsets) {
            for (const float sf : kSpeeds) {
                const float ang = baseAngle + off * kDeg2Rad;
                Vec2 cand{std::cos(ang), std::sin(ang)};
                cand = cand * (speed * sf);
                if (dot(cand, desiredVel) < 0.0f) {
                    continue;
                }
                if (!impl.mesh.contains(a.pos + cand * dt)) {
                    continue;
                }
                if (clear(margin, cand)) {
                    return cand;
                }
            }
        }
    }
    return std::nullopt;
}

// Post-decision verification of the committed substep swept segments. The
// candidate search in decideVelocity predicts against each other agent's
// snapshot-horizon swept segment, but the other's *committed* substep motion
// (chosen[j]*dt) can differ from its snapshot velocity when both agents swerve
// simultaneously, so the actual swept segments can overlap even though the
// horizon checks passed. This pass verifies the real committed segments and
// deterministically re-decides the higher-index agent of any overlapping pair
// against all others' committed segments (stop fallback). Terminates because a
// corrected agent either finds a clear candidate or stops.
void resolveCommitted(SimData& impl, const std::vector<Snapshot>& snap, std::vector<Vec2>& chosen, float dt)
{
    const std::size_t n = snap.size();
    for (std::size_t round = 0; round < n; ++round) {
        bool any = false;
        for (std::size_t i = 0; i < n && !any; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                const Vec2 a0 = snap[i].pos;
                const Vec2 b0 = snap[j].pos;
                const Vec2 a1 = a0 + chosen[i] * dt;
                const Vec2 b1 = b0 + chosen[j] * dt;
                const float need = snap[i].radius + snap[j].radius;
                if (internal::distSegSeg(a0, a1, b0, b1) < need) {
                    const std::size_t hi = j;
                    if (chosen[hi] != Vec2{0.0f, 0.0f}) {
                        auto cand = findCommittedCandidate(impl, snap, chosen, hi, dt);
                        chosen[hi] = cand ? *cand : Vec2{0.0f, 0.0f};
                    }
                    // A stopped agent cannot be corrected further; any residual
                    // overlap is accepted as SIM-010 robustness.
                    any = true;
                    break;
                }
            }
        }
        if (!any) {
            break;
        }
    }
}

// True if the whole swept displacement [from,to] stays inside the mesh (MSH-006
// containment). Membership is checked over the swept path, not just the end
// point, so an agent cannot tunnel through non-walkable space in one substep.
bool sweptContained(const NavMesh& mesh, Vec2 from, Vec2 to)
{
    auto pred = [&](Vec2 q) { return mesh.contains(q); };
    return internal::segmentContainedBy(from, to, pred);
}

// Applies the chosen velocity, clamping speed and never overshooting a waypoint
// or goal. Positions are kept inside the mesh, and the swept displacement must
// stay inside it too.
void applyMove(SimData::Agent& a, Vec2 chosen, float dt, const NavMesh& mesh)
{
    if (a.status != AgentStatus::Moving) {
        a.vel = {0.0f, 0.0f};
        return;
    }
    if (length(*a.goal - a.pos) <= a.arrivalRadius) {
        a.status = AgentStatus::Reached;
        a.vel = {0.0f, 0.0f};
        return;
    }

    const std::size_t idx = advanceRouteIndex(a);
    const Vec2 target = a.route[idx];

    // Clamp speed to max_speed.
    const float cl = length(chosen);
    if (cl > a.max_speed) {
        chosen = chosen * (a.max_speed / cl);
    }

    const Vec2 rem = target - a.pos;
    const float remLen = length(rem);
    const Vec2 disp = chosen * dt;

    Vec2 newpos;
    Vec2 newvel;
    // Snap to the waypoint only when the motion actually progresses past it
    // along the route direction (projection of the displacement onto the
    // waypoint vector covers the remaining distance). A large lateral swerve
    // whose magnitude happens to exceed the remaining distance must NOT snap;
    // otherwise the agent is teleported onto a waypoint it never reached.
    if (remLen > 0.0f && dot(disp, rem) >= remLen * remLen) {
        newpos = target;
        newvel = rem * (1.0f / dt);
        a.routeIndex = std::min(idx + 1, a.route.size() - 1);
    } else {
        newpos = a.pos + disp;
        newvel = chosen;
    }

    // Swept-path membership: the whole displacement must stay in the mesh.
    if (!sweptContained(mesh, a.pos, newpos)) {
        // Robustness fallback: never leave the mesh (SIM-012).
        a.vel = {0.0f, 0.0f};
        return;
    }

    a.pos = newpos;
    a.vel = newvel;

    if (length(*a.goal - a.pos) <= a.arrivalRadius) {
        a.status = AgentStatus::Reached;
        a.vel = {0.0f, 0.0f};
    }
}

} // anonymous namespace

Simulation::Simulation(NavMesh mesh) : m_impl(std::make_unique<Impl>(SimData{std::move(mesh), {}, 1}))
{
}

Simulation::~Simulation() = default;
Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

Result<AgentId> Simulation::add_agent(const AgentConfig& cfg)
{
    if (!internal::finite(cfg.position) || !internal::finite(cfg.radius) || !internal::finite(cfg.max_speed) ||
        !internal::finite(cfg.arrival_radius) || (cfg.goal.has_value() && !internal::finite(cfg.goal.value()))) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite configuration"});
    }
    if (!(cfg.radius > 0.0f) || !(cfg.max_speed > 0.0f)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "radius and max speed must be positive"});
    }
    if (!validArrivalRadius(cfg.arrival_radius)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "invalid arrival radius"});
    }
    if (!m_impl->data.mesh.contains(cfg.position)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "agent position outside mesh"});
    }
    if (cfg.goal.has_value() && !m_impl->data.mesh.contains(cfg.goal.value())) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "agent goal outside mesh"});
    }

    SimData::Agent a;
    a.id = m_impl->data.nextId;
    m_impl->data.nextId =
        (m_impl->data.nextId == std::numeric_limits<std::uint32_t>::max()) ? 1u : m_impl->data.nextId + 1u;
    a.pos = cfg.position;
    a.vel = {0.0f, 0.0f};
    a.radius = cfg.radius;
    a.max_speed = cfg.max_speed;
    a.arrivalRadius = effectiveArrival(cfg.radius, cfg.arrival_radius);
    a.goal = std::nullopt;
    a.status = AgentStatus::Idle;
    a.route.clear();
    a.routeIndex = 0;

    if (cfg.goal.has_value()) {
        assignGoal(m_impl->data, a, cfg.goal.value(), a.arrivalRadius);
    }

    m_impl->data.agents.push_back(std::move(a));
    return AgentId{m_impl->data.agents.back().id};
}

Result<void> Simulation::remove_agent(AgentId id)
{
    for (std::size_t i = 0; i < m_impl->data.agents.size(); ++i) {
        if (m_impl->data.agents[i].id == id.value) {
            m_impl->data.agents.erase(m_impl->data.agents.begin() + static_cast<std::ptrdiff_t>(i));
            return {};
        }
    }
    return std::unexpected(Error{ErrorCode::NotFound, "unknown agent id"});
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius)
{
    SimData::Agent* a = findAgent(m_impl->data, id);
    if (a == nullptr) {
        return std::unexpected(Error{ErrorCode::NotFound, "unknown agent id"});
    }
    if (!internal::finite(goal) || !internal::finite(arrival_radius)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite goal or arrival radius"});
    }
    if (!validArrivalRadius(arrival_radius)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "invalid arrival radius"});
    }
    if (!m_impl->data.mesh.contains(goal)) {
        return std::unexpected(Error{ErrorCode::OutsideMesh, "goal outside mesh"});
    }

    assignGoal(m_impl->data, *a, goal, effectiveArrival(a->radius, arrival_radius));
    return {};
}

Result<void> Simulation::clear_goal(AgentId id)
{
    SimData::Agent* a = findAgent(m_impl->data, id);
    if (a == nullptr) {
        return std::unexpected(Error{ErrorCode::NotFound, "unknown agent id"});
    }
    a->goal = std::nullopt;
    a->route.clear();
    a->routeIndex = 0;
    a->vel = {0.0f, 0.0f};
    a->status = AgentStatus::Idle;
    return {};
}

Result<void> Simulation::step(float seconds)
{
    if (!internal::finite(seconds)) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "non-finite duration"});
    }
    if (seconds < 0.0f) {
        return std::unexpected(Error{ErrorCode::InvalidArgument, "negative duration"});
    }
    if (seconds == 0.0f) {
        return {}; // no state change
    }

    const int n = static_cast<int>(std::ceil(seconds / kSubstep));
    for (int k = 0; k < n; ++k) {
        const float dt = std::min(kSubstep, std::max(0.0f, seconds - k * kSubstep));
        if (dt <= 0.0f) {
            break;
        }

        // Snapshot all live positions/velocities/radii before any update (SIM-010).
        std::vector<Snapshot> snap;
        snap.reserve(m_impl->data.agents.size());
        for (const auto& a : m_impl->data.agents) {
            snap.push_back({a.pos, a.vel, a.radius});
        }

        // Simultaneous velocity decisions from the snapshot.
        std::vector<Vec2> chosen(m_impl->data.agents.size());
        for (std::size_t i = 0; i < m_impl->data.agents.size(); ++i) {
            chosen[i] = decideVelocity(m_impl->data, m_impl->data.agents[i], i, snap, dt);
        }

        // Verify the committed substep swept segments pairwise and correct any
        // overlap deterministically (issue 3).
        resolveCommitted(m_impl->data, snap, chosen, dt);

        // Apply the chosen displacements.
        for (std::size_t i = 0; i < m_impl->data.agents.size(); ++i) {
            applyMove(m_impl->data.agents[i], chosen[i], dt, m_impl->data.mesh);
        }
    }
    return {};
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept
{
    const SimData::Agent* a = findAgent(m_impl->data, id);
    if (a == nullptr) {
        return std::nullopt;
    }
    AgentState st;
    st.position = a->pos;
    st.velocity = a->vel;
    st.radius = a->radius;
    st.max_speed = a->max_speed;
    st.goal = a->goal;
    st.status = a->status;
    return st;
}

std::size_t Simulation::agent_count() const noexcept
{
    return m_impl->data.agents.size();
}

} // namespace vwmini
