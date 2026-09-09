#include <vwmini/simulation.hpp>

#include "vwmini/internal/errors.hpp"
#include "vwmini/internal/predicates.hpp"
#include "vwmini/internal/vec2d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// Single-threaded disc-agent simulation over one immutable NavMesh
// (SIM-005..013).
//
// Ownership and identity: agents live in a slot vector; a hash map resolves
// public ids to slots. Ids are generated monotonically and never reused, so a
// removed id keeps returning `NotFound` forever (SIM-013). Removed slots are
// recycled through a free list.
//
// Stepping (SIM-008/010): `step` splits the duration into uniform substeps of
// at most 1/60 s (capped in count so huge finite durations stay responsive).
// Each substep runs in two phases: first every live agent is snapshotted and
// its velocity is planned from the snapshot alone, then all motion is applied.
// Agents without an interaction follow their route exactly by arc length,
// which lands precisely on waypoints and never overshoots; detoured agents
// integrate straight and fall back to route walking when the swept segment
// would leave the mesh, so positions stay contained (SIM-012). All planning
// math runs in double on float-derived values for determinism.

namespace vwmini {
namespace {

using internal::D2;
using internal::dist;
using internal::distance;
using internal::dot2;
using internal::is_finite;
using internal::length2;
using internal::make_error;
using internal::normalized2;
using internal::to_d;
using internal::to_vec;

/// Largest internal substep duration in seconds (SIM-008 allows subdivision).
constexpr double kMaxSubstep = 1.0 / 60.0;
/// Upper bound on substeps per `step` call. Durations beyond
/// `kMaxSubsteps * kMaxSubstep` (~68 s) still work: substeps become coarser,
/// while route walking keeps motion exact and bounded regardless of dt.
constexpr double kMaxSubsteps = 4096.0;

/// Prediction horizon for local avoidance, in seconds (SIM-011).
constexpr double kHorizon = 1.0;

/// Candidate deflections scored when an agent has neighbors: right side first
/// at each angle, full speed then half speed, and finally a full stop. The
/// desired velocity itself is always scored first as candidate zero.
constexpr std::array<std::pair<double, double>, 19> kDeflections = {{
    {-20.0, 1.0},  {20.0, 1.0},   {-45.0, 1.0}, {45.0, 1.0},  {-90.0, 1.0},
    {90.0, 1.0},   {-135.0, 1.0}, {135.0, 1.0}, {180.0, 1.0}, {-20.0, 0.5},
    {20.0, 0.5},   {-45.0, 0.5},  {45.0, 0.5},  {-90.0, 0.5}, {90.0, 0.5},
    {-135.0, 0.5}, {135.0, 0.5},  {180.0, 0.5}, {0.0, 0.0},
}};

/// One simulation-owned agent. Public snapshots (`AgentState`) expose a subset.
struct AgentRecord {
    /// Zero when the slot is free; otherwise the live public id value.
    std::uint32_t id_value{};
    Vec2 position{};
    /// Velocity executed by the most recent substep; zero unless `Moving`.
    Vec2 velocity{};
    float radius{};
    float max_speed{};
    /// Assigned goal; retained in every goal-bearing status (SIM-007/013).
    std::optional<Vec2> goal{};
    /// Effective (sentinel-resolved) arrival radius; meaningful while `goal` is set.
    float arrival_radius{};
    /// Route polyline from `find_path`; `route.back() == *goal` while `Moving`.
    std::vector<Vec2> route;
    /// Index in `route` of the point the agent currently steers toward.
    std::size_t next_waypoint{};
    AgentStatus status{AgentStatus::Idle};
};

/// Snapshot of one live agent at the start of a substep plus its planned
/// motion. Velocity planning reads snapshot values only (SIM-010).
struct FrameAgent {
    std::size_t slot{};
    Vec2 position{};
    Vec2 velocity{};
    float radius{};
    float max_speed{};
    /// Planned velocity for this substep; magnitude <= max_speed.
    Vec2 plan{};
    /// True when the plan deviates from pure route following.
    bool detoured{false};
};

/// SIM-006: `-1.0f` is the sole negative sentinel; every other negative value,
/// on either side of -1, is invalid. `-0.0f` counts as zero and is valid.
[[nodiscard]] bool valid_arrival_radius(float arrival) noexcept
{
    return arrival == -1.0f || arrival >= 0.0f;
}

/// Resolves the SIM-006 sentinel against the agent radius.
[[nodiscard]] float effective_arrival_radius(float specified, float agent_radius) noexcept
{
    return specified == -1.0f ? agent_radius : specified;
}

/// Desired route-following velocity: full speed toward the current waypoint.
/// Skips waypoints the agent sits on exactly so the direction stays defined.
/// Requires `rec.status == Moving` and a non-empty route.
[[nodiscard]] Vec2 desired_velocity(const AgentRecord& rec) noexcept
{
    std::size_t target = rec.next_waypoint;
    while (target + 1 < rec.route.size() && rec.route[target] == rec.position) {
        ++target;
    }
    return normalized(rec.route[target] - rec.position) * rec.max_speed;
}

/// True when the closed segment [a, b] lies entirely within the mesh. Queried
/// through the public path API because Simulation is not a friend of NavMesh
/// and cannot inspect its cells: find_path returns the direct two-point path
/// exactly when the segment is covered (SIM-001), and a bent corridor path or
/// an error otherwise. Successful paths preserve their endpoints (SIM-002), so
/// a two-point result is exactly {a, b}; the degenerate a == b case yields a
/// one-point path and reports false, which is harmless at the sole call site (a
/// zero plan walks the route by zero distance and stands still). Rejects
/// detours that would tunnel through a thin wall or across a concave notch even
/// though both endpoints are contained (SIM-012).
[[nodiscard]] bool segment_in_mesh(const NavMesh& mesh, Vec2 a, Vec2 b)
{
    const Result<Path> direct = find_path(mesh, a, b);
    return direct.has_value() && direct->points.size() == 2;
}

/// Lexicographic candidate ranking: feasible beats infeasible; among feasible
/// candidates, progress toward the desired velocity wins; among infeasible
/// ones, the smallest predicted encroachment wins. Comparisons are strict, so
/// ties keep the earlier candidate in scan order — deterministic selection.
/// Only candidates that would execute as planned are ranked; those leaving the
/// mesh are skipped before scoring (see `plan_velocity`).
struct PlanScore {
    bool feasible{false};
    /// Progress when feasible; predicted clearance deficit otherwise.
    double primary{};
    double progress{};

    [[nodiscard]] constexpr bool beats(const PlanScore& other) const noexcept
    {
        return std::tie(feasible, primary, progress) >
               std::tie(other.feasible, other.primary, other.progress);
    }
};

/// Predicted minimum distance over [0, horizon] between two points starting at
/// relative offset `w` and moving with constant relative velocity `z`.
[[nodiscard]] double predicted_min_distance(D2 w, D2 z, double horizon) noexcept
{
    const double zz = dot2(z, z);
    if (zz == 0.0) {
        return length2(w);
    }
    const double t = std::clamp(-dot2(w, z) / zz, 0.0, horizon);
    return length2(w + z * t);
}

/// Rotates a direction by `angle` radians; positive is counter-clockwise.
[[nodiscard]] D2 rotated(D2 direction, double angle) noexcept
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {direction.x * c - direction.y * s, direction.x * s + direction.y * c};
}

/// Per-substep motion plan for one agent.
struct MotionPlan {
    Vec2 velocity{};
    bool detoured{false};
};

} // namespace

struct Simulation::Impl {
    explicit Impl(NavMesh mesh_in) : mesh(std::move(mesh_in))
    {
    }

    // -- Agent lookup ---------------------------------------------------------

    /// Slot index of a live id, or empty for unknown/removed/zero ids.
    [[nodiscard]] std::optional<std::size_t> slot_of(std::uint32_t id_value) const noexcept
    {
        if (id_value == 0) {
            return std::nullopt;
        }
        const auto it = id_to_slot.find(id_value);
        if (it == id_to_slot.end()) {
            return std::nullopt;
        }
        return it->second;
    }

    /// Next monotonic non-zero id; ids are never reused (SIM-005/013).
    [[nodiscard]] std::uint32_t acquire_id() noexcept
    {
        const std::uint32_t id = next_id;
        ++next_id;
        if (next_id == 0) { // Unreachable in practice; keeps ids non-zero.
            next_id = 1;
        }
        return id;
    }

    // -- Lifecycle --------------------------------------------------------------

    [[nodiscard]] Result<AgentId> add_agent(const AgentConfig& config)
    {
        // Scalars first: a non-finite value is InvalidArgument even when a
        // point is also outside the mesh (NFR-006 ordering).
        if (!is_finite(config.position) || !is_finite(config.radius) ||
            !is_finite(config.max_speed) || !is_finite(config.arrival_radius) ||
            (config.goal.has_value() && !is_finite(*config.goal))) {
            return make_error(ErrorCode::InvalidArgument,
                              "agent config contains a non-finite value");
        }
        if (!(config.radius > 0.0f) || !(config.max_speed > 0.0f)) {
            return make_error(ErrorCode::InvalidArgument,
                              "agent radius and max speed must be positive");
        }
        if (!valid_arrival_radius(config.arrival_radius)) {
            return make_error(ErrorCode::InvalidArgument,
                              "arrival radius must be -1 (use agent radius) or non-negative");
        }
        if (!mesh.contains(config.position)) {
            return make_error(ErrorCode::OutsideMesh, "agent position is outside the navmesh");
        }
        if (config.goal.has_value() && !mesh.contains(*config.goal)) {
            return make_error(ErrorCode::OutsideMesh, "agent goal is outside the navmesh");
        }

        AgentRecord rec;
        rec.id_value = acquire_id();
        rec.position = config.position;
        rec.radius = config.radius;
        rec.max_speed = config.max_speed;

        std::size_t slot{};
        if (!free_slots.empty()) {
            slot = free_slots.back();
            free_slots.pop_back();
            slots[slot] = rec;
        } else {
            slot = slots.size();
            slots.push_back(rec);
        }
        id_to_slot.emplace(rec.id_value, slot);

        if (config.goal.has_value()) {
            // A disconnected goal is a valid NoPath state, not an error
            // (SIM-005); apply_goal only fails on unreachable find_path codes.
            if (Result<void> applied = apply_goal(slots[slot], *config.goal, config.arrival_radius);
                !applied.has_value()) {
                id_to_slot.erase(rec.id_value);
                slots[slot] = AgentRecord{};
                free_slots.push_back(slot);
                return std::unexpected(applied.error());
            }
        }
        return AgentId{rec.id_value};
    }

    [[nodiscard]] Result<void> remove_agent(AgentId id)
    {
        const std::optional<std::size_t> slot = slot_of(id.value);
        if (!slot.has_value()) {
            return make_error(ErrorCode::NotFound, "unknown or removed agent id");
        }
        id_to_slot.erase(id.value);
        slots[*slot] = AgentRecord{}; // Releases the route; id_value 0 marks free.
        free_slots.push_back(*slot);
        return {};
    }

    // -- Goals ------------------------------------------------------------------

    [[nodiscard]] Result<void> set_goal(AgentId id, Vec2 goal, float arrival_radius)
    {
        const std::optional<std::size_t> slot = slot_of(id.value);
        if (!slot.has_value()) {
            return make_error(ErrorCode::NotFound, "unknown or removed agent id");
        }
        if (!is_finite(goal) || !is_finite(arrival_radius)) {
            return make_error(ErrorCode::InvalidArgument, "goal and arrival radius must be finite");
        }
        if (!valid_arrival_radius(arrival_radius)) {
            return make_error(ErrorCode::InvalidArgument,
                              "arrival radius must be -1 (use agent radius) or non-negative");
        }
        if (!mesh.contains(goal)) {
            return make_error(ErrorCode::OutsideMesh, "goal is outside the navmesh");
        }
        // Validation completed; nothing above mutated the agent (SIM-008-style
        // transactional behavior applies to failures here too).
        return apply_goal(slots[*slot], goal, arrival_radius);
    }

    [[nodiscard]] Result<void> clear_goal(AgentId id)
    {
        const std::optional<std::size_t> slot = slot_of(id.value);
        if (!slot.has_value()) {
            return make_error(ErrorCode::NotFound, "unknown or removed agent id");
        }
        AgentRecord& rec = slots[*slot];
        rec.goal.reset();
        rec.arrival_radius = 0.0f;
        rec.route.clear();
        rec.next_waypoint = 0;
        rec.velocity = Vec2{};
        rec.status = AgentStatus::Idle;
        return {};
    }

    /// Establishes route and status for an already-validated in-mesh goal
    /// (SIM-005/007). Returns an error only for find_path failures other than
    /// NoPath, which two finite contained points cannot produce; propagating
    /// keeps the failure path honest instead of masking a broken invariant.
    [[nodiscard]] Result<void> apply_goal(AgentRecord& rec, Vec2 goal, float specified_arrival)
    {
        const float arrival = effective_arrival_radius(specified_arrival, rec.radius);
        Result<Path> path = find_path(mesh, rec.position, goal);
        // Surface the only failure two contained points can produce (a find_path
        // error other than NoPath) before mutating the record, so a failed
        // apply_goal leaves the agent untouched.
        if (!path.has_value() && path.error().code != ErrorCode::NoPath) {
            return std::unexpected(path.error());
        }
        rec.goal = goal;
        rec.arrival_radius = arrival;
        if (!path.has_value()) {
            // Disconnected but in-mesh: success with NoPath status (SIM-007).
            rec.route.clear();
            rec.next_waypoint = 0;
            rec.velocity = Vec2{};
            rec.status = AgentStatus::NoPath;
            return {};
        }
        rec.route = std::move(path->points);
        // route[0] is the current position; steer toward the next point.
        rec.next_waypoint = rec.route.size() > 1 ? 1 : 0;
        if (distance(rec.position, goal) <= double(arrival)) {
            // Already within the effective arrival radius (SIM-007).
            rec.velocity = Vec2{};
            rec.status = AgentStatus::Reached;
        } else {
            // Velocity is deliberately left unchanged here; step recomputes it
            // from executed motion. The contract fixes zero velocity only for
            // Reached, NoPath, and Idle transitions.
            rec.status = AgentStatus::Moving;
        }
        return {};
    }

    // -- Stepping -----------------------------------------------------------------

    [[nodiscard]] Result<void> step(float seconds)
    {
        if (!is_finite(seconds) || seconds < 0.0f) {
            return make_error(ErrorCode::InvalidArgument,
                              "step duration must be finite and non-negative");
        }
        if (seconds == 0.0f) { // Also covers -0.0f: success without changes.
            return {};
        }
        // Validated before any mutation, so an invalid step is transactional
        // (SIM-008). Uniform substeps keep motion deterministic.
        const double total = double(seconds);
        const double substeps = std::ceil(total / kMaxSubstep);
        const std::size_t n = std::size_t(std::clamp(substeps, 1.0, kMaxSubsteps));
        const double dt = total / double(n);
        for (std::size_t k = 0; k < n; ++k) {
            run_substep(dt);
        }
        return {};
    }

    void run_substep(double dt)
    {
        // Phase 1: snapshot every live agent (slot order) and plan velocities
        // from the snapshot alone (SIM-010).
        frame.clear();
        for (std::size_t s = 0; s < slots.size(); ++s) {
            const AgentRecord& rec = slots[s];
            if (rec.id_value == 0) {
                continue;
            }
            frame.push_back(FrameAgent{s, rec.position, rec.velocity, rec.radius, rec.max_speed,
                                       Vec2{}, false});
        }
        for (std::size_t i = 0; i < frame.size(); ++i) {
            const MotionPlan plan = plan_velocity(slots[frame[i].slot], i, dt);
            frame[i].plan = plan.velocity;
            frame[i].detoured = plan.detoured;
        }
        // Phase 2: apply the planned motion. Each application reads only its
        // own record and the immutable mesh.
        for (const FrameAgent& f : frame) {
            apply_movement(slots[f.slot], f.plan, f.detoured, dt);
        }
    }

    /// Plans this substep's velocity for one agent from the frame snapshot
    /// (SIM-010/011). Without neighbors inside the interaction horizon the plan
    /// is the exact route-following velocity. Otherwise a deterministic
    /// candidate set — desired velocity first, then fixed deflections at full
    /// and half speed, then a full stop — is scored against the frozen
    /// snapshot: deflections landing outside the mesh are skipped (a cheap
    /// subset of the stricter execution-time check), and a candidate is feasible
    /// when the predicted separation from every neighbor stays at or above the
    /// radius sum plus a margin absorbing one substep of simultaneous
    /// replanning. The winner is the feasible candidate with the most progress
    /// toward the goal, or, when none is feasible, the one maximizing
    /// predicted separation (so overlapping agents actively separate).
    [[nodiscard]] MotionPlan plan_velocity(const AgentRecord& rec, std::size_t self, double dt)
    {
        if (rec.status != AgentStatus::Moving || rec.route.size() < 2) {
            return {};
        }
        const Vec2 desired = desired_velocity(rec);
        const D2 desired_d = to_d(desired);
        if (length2(desired_d) == 0.0) {
            return MotionPlan{desired, false}; // Defensive: nothing to steer toward.
        }
        collect_neighbors(rec, self, dt);
        if (neighbors.empty()) {
            return MotionPlan{desired, false};
        }

        PlanScore best = score_candidate(rec, desired_d, desired_d, dt);
        D2 best_velocity = desired_d;
        bool best_detoured = false;
        const D2 dir = normalized2(desired_d);
        const double speed = double(rec.max_speed);
        const D2 self_position = to_d(rec.position);
        for (const auto& [angle_degrees, factor] : kDeflections) {
            const D2 candidate =
                rotated(dir, angle_degrees * std::numbers::pi / 180.0) * (factor * speed);
            // A deflection executes as straight integration, so one whose
            // substep landing leaves the mesh cannot serve as a detour. This
            // endpoint test is a deliberately cheap subset of the execution-time
            // check, which also rejects landings whose swept segment leaves the
            // mesh. The desired velocity walks the route and needs no test.
            if (!mesh.contains(to_vec(self_position + candidate * dt))) {
                continue;
            }
            const PlanScore score = score_candidate(rec, candidate, desired_d, dt);
            if (score.beats(best)) {
                best = score;
                best_velocity = candidate;
                best_detoured = true;
            }
        }
        return MotionPlan{to_vec(best_velocity), best_detoured};
    }

    /// Collects frame indices of agents that could interact with `rec` within
    /// the horizon; from farther away even head-on motion cannot close in time.
    void collect_neighbors(const AgentRecord& rec, std::size_t self, double dt)
    {
        neighbors.clear();
        const D2 p = to_d(rec.position);
        for (std::size_t i = 0; i < frame.size(); ++i) {
            if (i == self) {
                continue;
            }
            const FrameAgent& other = frame[i];
            const double reach =
                double(rec.radius) + double(other.radius) +
                (double(rec.max_speed) + double(other.max_speed)) * (kHorizon + 2.0 * dt);
            if (dist(p, to_d(other.position)) <= reach) {
                neighbors.push_back(i);
            }
        }
    }

    /// Scores one candidate velocity against the frozen snapshot: feasibility
    /// is the worst predicted clearance deficit over all neighbors, progress
    /// the alignment with the desired velocity. The caller only passes
    /// candidates that would execute as planned.
    [[nodiscard]] PlanScore score_candidate(const AgentRecord& rec, D2 candidate, D2 desired_d,
                                            double dt) const
    {
        const D2 self_position = to_d(rec.position);
        double worst_deficit = std::numeric_limits<double>::infinity();
        for (const std::size_t j : neighbors) {
            const FrameAgent& other = frame[j];
            const D2 w = self_position - to_d(other.position);
            const D2 z = candidate - to_d(other.velocity);
            // Margin absorbing simultaneous replanning (SIM-010): each agent's
            // executed substep displacement can deviate from its snapshot
            // prediction by up to 2 * max_speed * dt, so the executed pairwise
            // distance never dips below the sum of radii.
            const double required = double(rec.radius) + double(other.radius) +
                                    2.0 * (double(rec.max_speed) + double(other.max_speed)) * dt;
            worst_deficit =
                std::min(worst_deficit, predicted_min_distance(w, z, kHorizon) - required);
        }
        const bool feasible = worst_deficit >= 0.0;
        const double progress = dot2(candidate, desired_d);
        return PlanScore{feasible, feasible ? progress : worst_deficit, progress};
    }

    /// Applies one planned motion: exact arc-length walking along the route,
    /// or straight integration for detours accepted only when the whole swept
    /// segment stays in the mesh, with a route-walk fallback (SIM-008/012).
    /// Then resolves arrival (SIM-009) and records the executed velocity.
    void apply_movement(AgentRecord& rec, Vec2 plan, bool detoured, double dt)
    {
        if (rec.status != AgentStatus::Moving) {
            return; // Idle/Reached/NoPath agents never move; velocity stays zero.
        }
        const Vec2 old = rec.position;
        const double budget = length2(to_d(plan)) * dt;
        if (detoured) {
            const Vec2 trial = old + plan * float(dt);
            // Accept the detour only when the whole swept segment stays in the
            // mesh: a straight integration whose endpoint is contained could
            // still tunnel through a thin wall or across a concave notch.
            // segment_in_mesh subsumes the endpoint test (SIM-012); is_finite is
            // a cheap guard against calling find_path with garbage.
            if (is_finite(trial) && segment_in_mesh(mesh, old, trial)) {
                rec.position = trial;
                advance_crossed_waypoints(rec, old);
            } else {
                walk_route(rec, budget);
            }
        } else {
            walk_route(rec, budget);
        }
        if (distance(rec.position, *rec.goal) <= double(rec.arrival_radius)) {
            rec.status = AgentStatus::Reached;
            rec.velocity = Vec2{};
            return;
        }
        // Expose the velocity actually executed this substep (bounded by
        // max_speed), which is also what other agents predict from.
        const D2 delta = to_d(rec.position) - to_d(old);
        const double speed = std::clamp(length2(delta) / dt, 0.0, double(rec.max_speed));
        rec.velocity = to_vec(normalized2(delta) * speed);
    }

    /// Moves along the route by at most `budget` metres of arc length, landing
    /// exactly on waypoints and never past the final one (SIM-008). Route
    /// points are contained by construction (SIM-002), as is every position
    /// between consecutive ones.
    void walk_route(AgentRecord& rec, double budget) const
    {
        if (rec.route.size() < 2) {
            return;
        }
        const std::size_t last = rec.route.size() - 1;
        while (budget > 0.0 && rec.next_waypoint <= last) {
            const Vec2 target = rec.route[rec.next_waypoint];
            const double d = distance(rec.position, target);
            if (d > budget) {
                const D2 dir = normalized2(to_d(target) - to_d(rec.position));
                rec.position = to_vec(to_d(rec.position) + dir * budget);
                return;
            }
            rec.position = target;
            budget -= d;
            if (rec.next_waypoint == last) {
                return; // Landed on the goal; arrival is resolved by the caller.
            }
            ++rec.next_waypoint;
        }
    }

    /// Advances waypoints whose perpendicular line the agent crossed during a
    /// detour integration (route walking advances by exact landing instead).
    void advance_crossed_waypoints(AgentRecord& rec, Vec2 old) const
    {
        while (rec.next_waypoint + 1 < rec.route.size()) {
            const D2 a = to_d(rec.route[rec.next_waypoint]);
            const D2 seg = to_d(rec.route[rec.next_waypoint + 1]) - a;
            const bool was_before = dot2(to_d(old) - a, seg) <= 0.0;
            const bool is_after = dot2(to_d(rec.position) - a, seg) > 0.0;
            if (!was_before || !is_after) {
                return;
            }
            ++rec.next_waypoint;
        }
    }

    NavMesh mesh;
    /// All agent slots ever allocated; `id_value == 0` marks a free slot.
    std::vector<AgentRecord> slots;
    /// Recyclable slot indices, LIFO for deterministic reuse.
    std::vector<std::size_t> free_slots;
    /// Live id -> slot resolution; ids are unique for the simulation lifetime.
    std::unordered_map<std::uint32_t, std::size_t> id_to_slot;
    std::uint32_t next_id{1};
    /// Reused per-substep snapshot buffer (no per-substep allocation).
    std::vector<FrameAgent> frame;
    /// Reused per-plan buffer of interacting frame indices.
    std::vector<std::size_t> neighbors;
};

// -- Public forwarding -------------------------------------------------------
//
// A moved-from `Simulation` holds no mesh; it reports the least surprising
// error for each operation instead of dereferencing a null Impl.

Simulation::Simulation(NavMesh mesh) : m_impl(std::make_unique<Impl>(std::move(mesh)))
{
}

Simulation::~Simulation() = default;

Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

Result<AgentId> Simulation::add_agent(const AgentConfig& config)
{
    if (!m_impl) {
        return internal::make_error(ErrorCode::InvalidArgument, "simulation has been moved from");
    }
    return m_impl->add_agent(config);
}

Result<void> Simulation::remove_agent(AgentId id)
{
    if (!m_impl) {
        return internal::make_error(ErrorCode::NotFound, "simulation has been moved from");
    }
    return m_impl->remove_agent(id);
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius)
{
    if (!m_impl) {
        return internal::make_error(ErrorCode::NotFound, "simulation has been moved from");
    }
    return m_impl->set_goal(id, goal, arrival_radius);
}

Result<void> Simulation::clear_goal(AgentId id)
{
    if (!m_impl) {
        return internal::make_error(ErrorCode::NotFound, "simulation has been moved from");
    }
    return m_impl->clear_goal(id);
}

Result<void> Simulation::step(float seconds)
{
    if (!m_impl) {
        return internal::make_error(ErrorCode::InvalidArgument, "simulation has been moved from");
    }
    return m_impl->step(seconds);
}

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept
{
    if (!m_impl) {
        return std::nullopt;
    }
    const std::optional<std::size_t> slot = m_impl->slot_of(id.value);
    if (!slot.has_value()) {
        return std::nullopt;
    }
    const AgentRecord& rec = m_impl->slots[*slot];
    return AgentState{rec.position, rec.velocity, rec.radius, rec.max_speed, rec.goal, rec.status};
}

std::size_t Simulation::agent_count() const noexcept
{
    return m_impl ? m_impl->id_to_slot.size() : 0;
}

} // namespace vwmini
