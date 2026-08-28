// Simulation: agent lifecycle, goals, stepping, and deterministic local
// avoidance.

#include <vwmini/simulation.hpp>

#include "internal/geometry_detail.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace vwmini {

namespace {

using detail::kEpsilon;

// `step` is executed as equal substeps no longer than this bound, keeping
// per-step motion small so waypoints and goals are never overshot.
constexpr float kSubstepMax = 0.05f;
// Upper bound on the substep count: very large finite durations are clamped
// to this many substeps, so the float -> size_t conversion can never overflow
// and the loop always terminates. With kSubstepMax the covered duration is
// still astronomically larger than any simulation horizon.
constexpr std::size_t kMaxSubsteps = 65536;
// Pairs whose headings are within this cosine cone move in the same lane and
// use the anti-symmetric overtake rule instead of both steering right.
constexpr float kSameDirectionDot = 0.25f;
// Neighbours closer than radius sum + this range take part in avoidance.
constexpr float kAvoidRange = 2.0f;
// Predicted collisions are considered within this look-ahead horizon (seconds).
constexpr float kHorizon = 2.0f;
// Radial and tangential correction strengths for pairwise avoidance. The
// tangential (lateral) component dominates because separating the tracks
// laterally is what actually prevents a collision.
constexpr float kRadial = 0.6f;
constexpr float kTangential = 2.5f;
// Severity multiplier: the predicted required avoidance velocity is applied
// energetically because speed clamping would otherwise waste most of it.
constexpr float kAvoidanceStrength = 2.6f;
// Full turn in radians; used by the deterministic containment probe fan.
constexpr float kTwoPi = 6.283185307179586f;

[[nodiscard]] Error error(ErrorCode code, const char *reason) {
  return Error{code, std::string(reason)};
}

// Internal mutable agent record. Uniquely owned by one Simulation via slots.
struct Agent {
  Vec2 position{};
  Vec2 velocity{};
  float radius{0.25f};
  float max_speed{1.4f};
  std::optional<Vec2> goal{};
  float arrival_radius{0.25f};
  AgentStatus status{AgentStatus::Idle};
  std::vector<Vec2> route{}; // endpoint-preserving polyline; front == position
  std::size_t waypoint{0};   // index of the next route point to move toward
};

[[nodiscard]] bool valid_arrival_radius(float value) {
  // SIM-006: -1.0f is the single negative sentinel; -0.0f equals 0 and is
  // valid.
  return value == -1.0f || value >= 0.0f;
}

/// Applies the SIM-007 state transition for a validated in-mesh goal.
/// Precondition: `agent.arrival_radius` holds the effective arrival radius.
void apply_goal(Agent &agent, const NavMesh &mesh, Vec2 goal) {
  agent.goal = goal;
  agent.route.clear();
  agent.waypoint = 0;
  agent.velocity = {0.0f, 0.0f};
  if (length(goal - agent.position) <= agent.arrival_radius) {
    agent.status = AgentStatus::Reached; // already within the arrival radius
    return;
  }
  const auto path = find_path(mesh, agent.position, goal);
  if (!path) {
    agent.status = AgentStatus::NoPath; // in-mesh goal, disconnected component
    return;
  }
  agent.route = path->points;
  agent.waypoint = 1; // route.front() equals the position; move toward index 1
  agent.status = AgentStatus::Moving;
}

/// Advances all live agents by one deterministic substep. Decisions are made
/// from a snapshot of positions and velocities taken before any update, so the
/// avoidance is simultaneous (SIM-010). Iteration order is slot order.
/// `lane_side` pins the overtake side of each same-lane pair (slot i < slot j)
/// for the duration of an encounter, so the side can never flip mid-pass.
void advance(std::vector<std::optional<Agent>> &agents, const NavMesh &mesh,
             std::unordered_map<std::uint64_t, float> &lane_side, float dt) {
  const std::size_t n = agents.size();
  std::vector<Vec2> positions(n);
  std::vector<Vec2> velocities(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (agents[i]) {
      positions[i] = agents[i]->position;
      velocities[i] = agents[i]->velocity;
    }
  }

  // 1. Desired velocities: straight toward the next waypoint, never faster
  //    than required to reach it within this substep (no overshoot).
  std::vector<Vec2> desired(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (!agents[i] || agents[i]->status != AgentStatus::Moving) {
      desired[i] = {0.0f, 0.0f};
      continue;
    }
    const Agent &a = *agents[i];
    const Vec2 target =
        (a.waypoint < a.route.size()) ? a.route[a.waypoint] : *a.goal;
    const Vec2 to_target = target - positions[i];
    const float d = length(to_target);
    if (d <= 0.0f) {
      desired[i] = {0.0f, 0.0f};
      continue;
    }
    const float speed = std::min(a.max_speed, d / dt);
    desired[i] = to_target * (speed / d);
  }

  // 2. Pairwise avoidance from the snapshot (order i < j). The correction
  //    blends a push directly away from the neighbour with a tangential steer
  //    on the right of each moving agent's own heading. Non-moving agents
  //    never receive a deflection, so terminal states stay exactly stable.
  for (std::size_t i = 0; i < n; ++i) {
    if (!agents[i]) {
      continue;
    }
    const Agent &ai = *agents[i];
    const bool di_moving = ai.status == AgentStatus::Moving;
    for (std::size_t j = i + 1; j < n; ++j) {
      const std::uint64_t key =
          (static_cast<std::uint64_t>(i) << 32) | static_cast<std::uint64_t>(j);
      if (!agents[j]) {
        lane_side.erase(key);
        continue;
      }
      const Agent &aj = *agents[j];
      if (!di_moving && aj.status != AgentStatus::Moving) {
        continue;
      }
      const Vec2 rel_pos = positions[i] - positions[j];
      const float d = length(rel_pos);
      const float r_ab = ai.radius + aj.radius;
      if (d > r_ab + kAvoidRange) {
        lane_side.erase(key);
        continue;
      }
      Vec2 radial = (d > 1e-6f) ? rel_pos * (1.0f / d) : Vec2{1.0f, 0.0f};
      float severity = 0.0f;
      if (d < r_ab) {
        severity = (r_ab - d) * 2.0f; // already overlapping: strong push
      } else {
        // Envelope push: a pair this close is always pushed apart
        // steadily, so it cannot coast back into contact between
        // predicted-approach corrections.
        severity = std::max(severity, (r_ab + 0.35f - d) * 2.5f);
        const Vec2 rel_vel = velocities[i] - velocities[j];
        const double approaching = detail::dot2(rel_pos, rel_vel);
        if (approaching < 0.0) {
          const double rel_speed2 = detail::dot2(rel_vel, rel_vel);
          if (rel_speed2 > 1e-9) {
            const double t_c = -approaching / rel_speed2;
            if (t_c <= kHorizon) {
              const Vec2 closest = rel_pos + rel_vel * static_cast<float>(t_c);
              const float closest_d = length(closest);
              if (closest_d < r_ab) {
                severity = (r_ab - closest_d) / static_cast<float>(t_c + 1e-6);
              }
            }
          }
        }
      }
      if (severity <= 0.0f) {
        continue;
      }
      // Traffic-rule steering: a push directly away from the neighbour plus
      // a lateral steer. Crossing/head-on pairs both steer to the right-hand
      // side of their own heading, which is world-opposite for a head-on
      // meeting and so separates them; the heading-based side never flips
      // mid-approach. Same-lane pairs (headings in one cone) instead use an
      // anti-symmetric rule: the follower veers right and the leader left.
      // Both steering to their own right would push same-direction agents to
      // the same world side, leaving only the radial push to separate an
      // overtaking pair.
      const auto steer = [&](Vec2 heading, float radial_sign, float tang_side) {
        const float dl = length(heading);
        if (dl <= 1e-6f) {
          // Headingless: a pure radial push (normalized) is all there is.
          return radial * (radial_sign * severity * kAvoidanceStrength);
        }
        const Vec2 right{heading.y / dl, -heading.x / dl};
        const Vec2 blend = radial * (kRadial * radial_sign) +
                           right * (kTangential * tang_side);
        const float bl = length(blend);
        return (bl > 0.0f) ? blend * (severity * kAvoidanceStrength / bl)
                           : Vec2{};
      };
      if (di_moving) {
        const Vec2 hi = desired[i];
        const Vec2 hj = desired[j];
        const float hdi = length(hi);
        const float hdj = length(hj);
        bool same_lane = false;
        float side_i = 1.0f;
        if (hdi > 1e-6f && hdj > 1e-6f &&
            detail::dot2(hi, hj) / (hdi * hdj) > kSameDirectionDot) {
          same_lane = true;
          // Pin the overtake side at the first in-range contact with the
          // follower (the agent whose counterpart lies ahead along the shared
          // heading) taking its own right-hand side, and hold it for the
          // whole encounter. Re-evaluating it every substep would flip the
          // assignment around the passing moment and squeeze the pair
          // together exactly when they are closest.
          const auto it = lane_side.find(key);
          if (it != lane_side.end()) {
            side_i = it->second;
          } else {
            const Vec2 h_shared = normalized(hi + hj);
            side_i = (detail::dot2(positions[j] - positions[i], h_shared) > 0.0)
                         ? 1.0f
                         : -1.0f;
            lane_side.emplace(key, side_i);
          }
        } else {
          lane_side.erase(key);
        }
        desired[i] = desired[i] + steer(hi, 1.0f, same_lane ? side_i : 1.0f);
        if (aj.status == AgentStatus::Moving) {
          desired[j] =
              desired[j] + steer(hj, -1.0f, same_lane ? -side_i : 1.0f);
        }
      } else if (aj.status == AgentStatus::Moving) {
        desired[j] = desired[j] + steer(desired[j], -1.0f, 1.0f);
      }
    }
  }

  // 3. Apply: clamp to max speed, cap at the next waypoint, stay contained.
  for (std::size_t i = 0; i < n; ++i) {
    if (!agents[i]) {
      continue;
    }
    Agent &a = *agents[i];
    Vec2 v = desired[i];
    const float speed = length(v);
    if (speed > a.max_speed && speed > 0.0f) {
      v = v * (a.max_speed / speed);
    }
    Vec2 move = v * dt;
    if (a.status == AgentStatus::Moving) {
      const Vec2 to_target = (a.waypoint < a.route.size())
                                 ? a.route[a.waypoint] - a.position
                                 : *a.goal - a.position;
      const float d = length(to_target);
      const float m = length(move);
      if (d > 0.0f && m > d) {
        move = move * (d / m); // never pass the waypoint/goal this substep
      }
    }
    // Containment with wall sliding: an unobstructed move is applied as-is; a
    // blocked one first shrinks to the furthest in-mesh point, and when that
    // leaves no progress the agent slides along the boundary instead of
    // stopping dead (an agent pressed against a wall must keep the tangential
    // part of its motion). The inward direction is estimated from a fixed fan
    // of containment probes around the contact point, so the slide direction
    // is a property of the mesh rather than of the world axes (rotation
    // invariant) and needs no triangle or edge lookup.
    Vec2 new_pos = a.position + move;
    if (!mesh.contains(new_pos)) {
      float t = 1.0f;
      bool hit = false;
      for (int k = 0; k < 12; ++k) {
        if (mesh.contains(a.position + move * t)) {
          hit = true;
          break;
        }
        t *= 0.5f;
      }
      const Vec2 contact = hit ? (a.position + move * t) : a.position;
      new_pos = contact; // best achievable along the blocked direction
      Vec2 inward{};
      constexpr int kFan = 36;
      const float radius = 8.0f * kEpsilon;
      for (int k = 0; k < kFan; ++k) {
        const float angle =
            static_cast<float>(k) * kTwoPi / static_cast<float>(kFan);
        const Vec2 dir{std::cos(angle), std::sin(angle)};
        if (mesh.contains(contact + dir * radius)) {
          inward = inward + dir; // inside half-plane
        }
      }
      const float in_len = length(inward);
      if (in_len > 1e-6f) {
        const Vec2 n = inward * (1.0f / in_len);
        const Vec2 t1{-n.y, n.x};
        const Vec2 t2{n.y, -n.x};
        const Vec2 tangent =
            (detail::dot2(move, t1) >= detail::dot2(move, t2)) ? t1 : t2;
        const float along = static_cast<float>(detail::dot2(move, tangent));
        if (along > 1e-6f) {
          Vec2 slid = a.position + tangent * along;
          if (!mesh.contains(slid)) {
            // Curved boundary or a corner: shrink the slide, never go
            // backwards.
            float s = 1.0f;
            bool ok = false;
            for (int k = 0; k < 12; ++k) {
              if (mesh.contains(a.position + tangent * (along * s))) {
                ok = true;
                break;
              }
              s *= 0.5f;
            }
            slid = ok ? (a.position + tangent * (along * s)) : contact;
          }
          new_pos = slid;
        }
      }
    }
    // Defensive: refuse to adopt any non-finite position.
    if (!detail::is_finite(new_pos)) {
      new_pos = a.position;
    }
    a.position = new_pos;
    a.velocity = v; // executed decision, bounded by max speed

    if (a.status == AgentStatus::Moving && a.waypoint < a.route.size() &&
        length(a.position - a.route[a.waypoint]) <= kEpsilon) {
      ++a.waypoint;
    }
    if (a.status == AgentStatus::Moving && a.goal &&
        length(a.position - *a.goal) <= a.arrival_radius) {
      a.status = AgentStatus::Reached; // SIM-009
      a.velocity = {0.0f, 0.0f};
    }
  }
}

/// Returns a pointer to the live agent for `id`, or nullptr.
[[nodiscard]] Agent *find_slot(std::vector<std::optional<Agent>> &agents,
                               AgentId id) noexcept {
  if (id.value == 0 || static_cast<std::size_t>(id.value) > agents.size()) {
    return nullptr;
  }
  std::optional<Agent> &slot = agents[static_cast<std::size_t>(id.value - 1)];
  return slot ? &*slot : nullptr;
}

} // namespace

struct Simulation::Impl {
  explicit Impl(NavMesh m) : mesh(std::move(m)) {}
  NavMesh mesh;
  // Slots indexed by id - 1; removed agents leave an empty slot and their
  // ids are never reused, so a removed id stays NotFound forever.
  std::vector<std::optional<Agent>> agents;
  std::uint32_t next_id = 1;
  std::size_t live_count = 0;
  // Pinned overtake sides for same-lane pairs in contact (slot i < slot j);
  // see `advance`. Removed when an encounter ends.
  std::unordered_map<std::uint64_t, float> lane_side;
};

Simulation::Simulation(NavMesh mesh)
    : m_impl(std::make_unique<Impl>(std::move(mesh))) {}

Simulation::~Simulation() = default;
Simulation::Simulation(Simulation &&) noexcept = default;
Simulation &Simulation::operator=(Simulation &&) noexcept = default;

Result<AgentId> Simulation::add_agent(const AgentConfig &config) {
  Impl &impl = *m_impl;
  if (!detail::is_finite(config.position) ||
      !detail::is_finite(config.radius) ||
      !detail::is_finite(config.max_speed) ||
      !detail::is_finite(config.arrival_radius) ||
      (config.goal && !detail::is_finite(*config.goal))) {
    return std::unexpected(
        error(ErrorCode::InvalidArgument, "non-finite configuration"));
  }
  if (config.radius <= 0.0f || config.max_speed <= 0.0f) {
    return std::unexpected(error(ErrorCode::InvalidArgument,
                                 "radius and max speed must be positive"));
  }
  if (!valid_arrival_radius(config.arrival_radius)) {
    return std::unexpected(error(ErrorCode::InvalidArgument,
                                 "arrival radius must be -1, 0, or positive"));
  }
  if (!impl.mesh.contains(config.position)) {
    return std::unexpected(
        error(ErrorCode::OutsideMesh, "position outside the mesh"));
  }
  if (config.goal && !impl.mesh.contains(*config.goal)) {
    return std::unexpected(
        error(ErrorCode::OutsideMesh, "goal outside the mesh"));
  }

  Agent agent;
  agent.position = config.position;
  agent.radius = config.radius;
  agent.max_speed = config.max_speed;
  agent.arrival_radius =
      (config.arrival_radius == -1.0f) ? config.radius : config.arrival_radius;
  if (config.goal) {
    apply_goal(agent, impl.mesh, *config.goal);
  }

  const std::uint32_t id = impl.next_id++;
  impl.agents.emplace_back(std::move(agent));
  ++impl.live_count;
  return AgentId{id};
}

Result<void> Simulation::remove_agent(AgentId id) {
  if (find_slot(m_impl->agents, id)) {
    m_impl->agents[static_cast<std::size_t>(id.value - 1)].reset();
    --m_impl->live_count;
    return {};
  }
  return std::unexpected(error(ErrorCode::NotFound, "unknown agent id"));
}

Result<void> Simulation::set_goal(AgentId id, Vec2 goal, float arrival_radius) {
  Agent *slot = find_slot(m_impl->agents, id);
  if (!slot) {
    return std::unexpected(error(ErrorCode::NotFound, "unknown agent id"));
  }
  if (!detail::is_finite(goal) || !detail::is_finite(arrival_radius)) {
    return std::unexpected(
        error(ErrorCode::InvalidArgument, "non-finite goal or arrival radius"));
  }
  if (!valid_arrival_radius(arrival_radius)) {
    return std::unexpected(error(ErrorCode::InvalidArgument,
                                 "arrival radius must be -1, 0, or positive"));
  }
  if (!m_impl->mesh.contains(goal)) {
    return std::unexpected(
        error(ErrorCode::OutsideMesh, "goal outside the mesh"));
  }
  slot->arrival_radius =
      (arrival_radius == -1.0f) ? slot->radius : arrival_radius;
  apply_goal(*slot, m_impl->mesh, goal);
  return {};
}

Result<void> Simulation::clear_goal(AgentId id) {
  Agent *slot = find_slot(m_impl->agents, id);
  if (!slot) {
    return std::unexpected(error(ErrorCode::NotFound, "unknown agent id"));
  }
  slot->goal.reset();
  slot->route.clear();
  slot->waypoint = 0;
  slot->velocity = {0.0f, 0.0f};
  slot->status = AgentStatus::Idle;
  return {};
}

Result<void> Simulation::step(float seconds) {
  if (!detail::is_finite(seconds) || seconds < 0.0f) {
    return std::unexpected(error(ErrorCode::InvalidArgument,
                                 "duration must be finite and non-negative"));
  }
  if (seconds == 0.0f || m_impl->live_count == 0) {
    return {}; // success with no state change
  }
  // Compute the substep count in double precision and clamp it. A direct
  // float -> size_t cast of ceil(seconds / kSubstepMax) overflows for huge
  // finite durations (e.g. 3.4e38 / 0.05 is inf in float), and an unbounded
  // substep count would require an unbounded loop.
  const double raw_substeps =
      std::ceil(static_cast<double>(seconds) / kSubstepMax);
  const std::size_t substeps =
      (raw_substeps >= static_cast<double>(kMaxSubsteps))
          ? kMaxSubsteps
          : std::max<std::size_t>(1, static_cast<std::size_t>(raw_substeps));
  const float dt = seconds / static_cast<float>(substeps);
  for (std::size_t s = 0; s < substeps; ++s) {
    advance(m_impl->agents, m_impl->mesh, m_impl->lane_side, dt);
    // Once no agent is moving, further substeps cannot change any state
    // (advance is a no-op for Idle/Reached/NoPath agents), so stop early.
    // This is bit-identical to running the remaining substeps.
    bool any_moving = false;
    for (const auto &slot : m_impl->agents) {
      if (slot && slot->status == AgentStatus::Moving) {
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

std::optional<AgentState> Simulation::agent(AgentId id) const noexcept {
  if (id.value == 0 ||
      static_cast<std::size_t>(id.value) > m_impl->agents.size()) {
    return std::nullopt;
  }
  const std::optional<Agent> &slot =
      m_impl->agents[static_cast<std::size_t>(id.value - 1)];
  if (!slot) {
    return std::nullopt;
  }
  const Agent &a = *slot;
  return AgentState{a.position,  a.velocity, a.radius,
                    a.max_speed, a.goal,     a.status};
}

std::size_t Simulation::agent_count() const noexcept {
  return m_impl->live_count;
}

} // namespace vwmini
