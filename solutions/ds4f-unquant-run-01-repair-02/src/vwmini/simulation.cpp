// Simulation: agent lifecycle, goals, stepping, and deterministic local
// avoidance.

#include <vwmini/simulation.hpp>

#include "internal/geometry_detail.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
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
// Neighbours closer than radius sum + this range take part in avoidance.
constexpr float kAvoidRange = 2.0f;
// Predicted collisions are considered within this look-ahead horizon (seconds).
constexpr float kHorizon = 2.0f;
// Pairs closer than radius sum + this margin are always in conflict and are
// forced apart at the deficit rate, so passes hold at least the sum plus this
// envelope rather than drifting down to hard contact. The margin is
// deliberately narrow: avoidance is otherwise driven purely by predicted
// collisions, so a parked neighbour is never a permanent repulsion well and
// short passes are not deflected.
constexpr float kEnvelopeMargin = 0.09f;
// The escape velocity is scaled by this gain before the half-plane split.
// A gain above 1 overshoots the velocity-obstacle boundary while the
// constraint is active, building the lateral offset that carries the pair
// cleanly past each other on the return swing.
constexpr float kEscapeGain = 2.0f;
// Full turn in radians; used by the deterministic containment probe fan.
constexpr float kTwoPi = 6.283185307179586f;

// A velocity half-plane constraint v.n >= low, produced by one pairwise
// encounter. `n` is the unit escape direction (the minimal change to the
// relative velocity that clears the predicted collision) and `low` the
// minimum component a moving agent must keep along it.
struct Constraint {
  Vec2 n{};
  float low{0.0f};
};

/// Returns the point of `disc(max_speed) ∩ {v.n >= low for all constraints}`
/// closest to `pref` (an ORCA-style projection). Candidates are the preferred
/// velocity itself, the projection onto each constraint line, and every pair
/// of crossing constraint lines, each clamped into the disc; the feasible
/// candidate nearest to `pref` wins. If the feasible set is empty (heavily
/// congested encounters) every violated constraint pushes along its normal
/// with its full deficit, clamped to the disc, so the agent still moves
/// decisively away from each threat instead of freezing.
[[nodiscard]] Vec2 constrained_velocity(Vec2 pref, float max_speed,
                                        const std::vector<Constraint> &cons) {
  const float R = max_speed;
  if (cons.empty()) {
    return pref;
  }
  const auto feasible = [&](Vec2 v) {
    if (!detail::is_finite(v)) {
      return false;
    }
    if (length(v) > R + 1e-4f) {
      return false;
    }
    for (const Constraint &c : cons) {
      if (static_cast<float>(detail::dot2(v, c.n)) < c.low - 1e-4f) {
        return false;
      }
    }
    return true;
  };
  if (feasible(pref)) {
    return pref;
  }
  const auto clamp_disc = [R](Vec2 v) {
    const float l = length(v);
    return (l > R && l > 1e-12f) ? v * (R / l) : v;
  };
  float best_d2 = std::numeric_limits<float>::infinity();
  Vec2 best{};
  const auto offer = [&](Vec2 v) {
    if (!feasible(v)) {
      return;
    }
    const Vec2 diff = v - pref;
    const float d2 = static_cast<float>(detail::dot2(diff, diff));
    if (d2 < best_d2) {
      best_d2 = d2;
      best = v;
    }
  };
  offer(clamp_disc(pref));
  for (const Constraint &c : cons) {
    offer(clamp_disc(
        pref + (c.low - static_cast<float>(detail::dot2(pref, c.n))) * c.n));
  }
  for (std::size_t k = 0; k < cons.size(); ++k) {
    for (std::size_t l = k + 1; l < cons.size(); ++l) {
      const Constraint &a = cons[k];
      const Constraint &b = cons[l];
      const double det = detail::cross2(a.n, b.n);
      if (std::abs(det) < 1e-6) {
        continue; // parallel lines: no corner candidate
      }
      // Intersection of a.n.x*X + a.n.y*Y = a.low and the same for b.
      const float x = static_cast<float>((static_cast<double>(a.low) * b.n.y -
                                          static_cast<double>(b.low) * a.n.y) /
                                         det);
      const float y = static_cast<float>((static_cast<double>(a.n.x) * b.low -
                                          static_cast<double>(b.n.x) * a.low) /
                                         det);
      offer(clamp_disc(Vec2{x, y}));
    }
  }
  if (std::isfinite(best_d2)) {
    return best;
  }
  Vec2 push{};
  for (const Constraint &c : cons) {
    const float deficit = c.low - static_cast<float>(detail::dot2(pref, c.n));
    if (deficit > 0.0f) {
      push = push + c.n * deficit;
    }
  }
  return clamp_disc(pref + push);
}

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

/// Escape state for one pair (i < j), kept across substeps so that a pair on
/// a predicted collision course keeps evading until it has *physically*
/// separated. Without this stickiness a symmetric crossing oscillates: the
/// frame after a deflection the (now escaping) relative velocity leaves the
/// cone, the constraint is released, both agents snap back onto the goal line
/// (undoing the evasion), and the next frame re-enters the cone -- the pair
/// averages out at a pass narrower than required.
struct PairEscape {
  std::uint8_t active = 0;
  Vec2 n{};          // escape normal shared by the pair (i gets +n, j gets -n)
  float half = 0.0f; // each agent must keep >= half of progress along +-n
};

/// Advances all live agents by one deterministic substep. Decisions are made
/// from a snapshot of positions and velocities taken before any update, so the
/// avoidance is simultaneous (SIM-010). Iteration order is slot order.
void advance(std::vector<std::optional<Agent>> &agents, const NavMesh &mesh,
             float dt, std::vector<PairEscape> &pair_esc) {
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
    // The distance/speed arithmetic is done in double: `d / dt` underflows to
    // zero in float for tiny distances and huge (but finite) durations, which
    // would freeze an agent short of its goal forever.
    const float speed = static_cast<float>(
        std::min(static_cast<double>(a.max_speed),
                 static_cast<double>(d) / static_cast<double>(dt)));
    desired[i] = to_target * (speed / d);
  }

  // 2. Pairwise avoidance from the snapshot (order i < j). For every pair on
  //    a predicted collision course (or already overlapping), each moving
  //    agent receives an ORCA-style half-plane constraint on its velocity:
  //    v.n >= low. The unit normal is the escape direction, i.e. the minimal
  //    change to the relative velocity that clears the collision (the
  //    boundary of the velocity-obstacle cone), so the two agents are pushed
  //    apart along the geometry of the encounter and never toward each
  //    other; because the agents use opposite sides of the same constraint,
  //    satisfying both destroys the predicted collision. Non-moving agents
  //    never receive a constraint, so terminal states stay exactly stable.
  std::vector<std::vector<Constraint>> cons(n);
  if (pair_esc.size() != n * n) {
    pair_esc.assign(n * n, PairEscape{});
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (!agents[i]) {
      continue;
    }
    const bool di_moving = agents[i]->status == AgentStatus::Moving;
    for (std::size_t j = i + 1; j < n; ++j) {
      if (!agents[j]) {
        continue;
      }
      const bool dj_moving = agents[j]->status == AgentStatus::Moving;
      if (!di_moving && !dj_moving) {
        continue;
      }
      const float r_ab = agents[i]->radius + agents[j]->radius;
      // The cone and the overlap resolution aim at this enlarged separation so
      // a contact-free pass keeps a small but real envelope (strict probes
      // require min_gap >= r_ab exactly; landing on the boundary in float is
      // one ulp short).
      const float r_cone = r_ab + kEnvelopeMargin;
      const Vec2 rel_pos = positions[i] - positions[j];
      const float d = length(rel_pos);
      PairEscape &e = pair_esc[i * n + j];
      if (d > r_ab + kAvoidRange) {
        e = PairEscape{}; // far apart: nothing to remember
        continue;
      }
      const Vec2 rel_vel = velocities[i] - velocities[j];
      Vec2 n{};          // escape normal (unit), from j toward i
      float half = 0.0f; // each agent must gain >= half along +-n
      if (d < r_ab) {
        // Already overlapping: resolve radially within this substep.
        n = (d > 1e-6f) ? rel_pos * (1.0f / d) : Vec2{1.0f, 0.0f};
        half = 0.5f * (r_cone - d) / dt;
        e = PairEscape{1, n, half};
      } else {
        // Predicted collision: the relative motion on the current courses
        // carries the pair closer than the (enlarged) radii sum within the
        // horizon. Pairs that are merely nearby but separating are left
        // alone: a wide always-on repulsion would be a permanent well around
        // parked agents and would shove a cornering agent into the wall
        // instead of letting it pass.
        const double along = detail::dot2(rel_pos, rel_vel);
        bool initiate = false;
        if (along < 0.0) {
          const double rel_speed2 = detail::dot2(rel_vel, rel_vel);
          if (rel_speed2 > 1e-12) {
            const double t_c = -along / rel_speed2;
            if (t_c <= kHorizon) {
              const Vec2 closest = rel_pos + rel_vel * static_cast<float>(t_c);
              if (length(closest) < r_cone) {
                initiate = true;
              }
            }
          }
        }
        // Pairs already inside the envelope are always in conflict: the
        // horizon gate alone would let a slow pair creep toward contact with
        // t_c beyond the horizon, sustaining a stale escape forever (the
        // stored half-plane amounts to nothing once the deficit closed).
        if (d < r_cone) {
          initiate = true;
        }
        if (!e.active && !initiate) {
          continue; // not avoiding and no predicted collision
        }
        // Sticky release: once the pair has *physically* separated (distance
        // at/above the envelope and relative motion separating), drop the
        // constraint. Until then the escape keeps being enforced even on
        // frames where the instantaneous relative velocity already points
        // outside the cone, so the agents cannot snap back onto the course.
        if (e.active && along >= 0.0f &&
            static_cast<float>(length(rel_pos)) >= r_cone) {
          e = PairEscape{};
          continue;
        }
        if (e.active && !initiate) {
          // Sustain the previous escape: the relative velocity has already
          // left the cone; keep the stored half-plane instead of letting the
          // agents revert to their preferred (collision) courses.
          n = e.n;
          half = e.half;
        } else {
          // The boundary of the velocity obstacle is the pair of rays from
          // the origin at +-alpha around the direction from i toward j (a
          // relative velocity on the axis carries i straight onto j's
          // position). The escape velocity is the boundary point nearest to
          // the current relative velocity (the minimal change that clears the
          // collision); ties are broken deterministically toward the +alpha
          // ray.
          const float axis_x = -rel_pos.x / d;
          const float axis_y = -rel_pos.y / d;
          const float sin_a = r_cone / d;
          const float cos_a = std::sqrt(std::max(0.0f, 1.0f - sin_a * sin_a));
          const float rx_p = axis_x * cos_a - axis_y * sin_a;
          const float ry_p = axis_x * sin_a + axis_y * cos_a;
          const float rx_m = axis_x * cos_a + axis_y * sin_a;
          const float ry_m = -axis_x * sin_a + axis_y * cos_a;
          const float tp = std::max(0.0f, static_cast<float>(detail::dot2(
                                              rel_vel, Vec2{rx_p, ry_p})));
          const float tm = std::max(0.0f, static_cast<float>(detail::dot2(
                                              rel_vel, Vec2{rx_m, ry_m})));
          const Vec2 q_plus{rx_p * tp, ry_p * tp};
          const Vec2 q_minus{rx_m * tm, ry_m * tm};
          const Vec2 u_p = q_plus - rel_vel;
          const Vec2 u_m = q_minus - rel_vel;
          const float len_p = length(u_p);
          const float len_m = length(u_m);
          if (len_p <= 1e-6f && len_m <= 1e-6f) {
            continue; // nothing to change (numerically on the boundary)
          }
          // The tangent escape keeps the *instantaneous* relative velocity on
          // a boundary ray (a chord aiming at r_cone from the current
          // center). Re-aimed every substep at the moved partner, that chord
          // drifts inward and passes land on the hard-contact circle instead
          // of the envelope. Once the pair is inside the envelope
          // (d < r_cone) the constraint therefore switches to a *radial*
          // escape that forces the separation deficit to close at exactly
          // (r_cone - d) per second, so the pass cannot fall below the
          // envelope.
          //
          // Except for the apex case: a relative velocity along the axis
          // (the pair charging straight onto each other) projects onto
          // neither boundary ray and the minimal tangential change would be
          // u = -vr -- a pure reversal that stalls exactly-equal agents in
          // the classic alternating reverse/re-commit cycle. Substitute a
          // deterministic *lateral* escape (perpendicular to the axis,
          // toward the +alpha side): the pair veers apart sideways, which
          // separates them *and* accumulates the offset needed for the
          // crossing, with no asymmetry between agents.
          const float vr_len = static_cast<float>(length(rel_vel));
          const bool apex =
              vr_len > 1e-4f && tp <= 1e-4f * vr_len && tm <= 1e-4f * vr_len;
          if (sin_a >= 1.0f - 1e-6f) {
            if (apex) {
              n = Vec2{rx_p, ry_p}; // unit: rotate of the axis by +90 degrees
            } else {
              n = rel_pos * (1.0f / d); // straight away from the partner
            }
            half = 0.5f * (r_cone - d) / dt * kEscapeGain;
            e = PairEscape{1, n, half};
          } else {
            const Vec2 u = (len_p <= len_m) ? u_p : u_m;
            const float ul = (len_p <= len_m) ? len_p : len_m;
            n = u * (1.0f / ul);
            half = 0.5f * ul * kEscapeGain;
            e = PairEscape{1, n, half};
          }
        }
      }
      // The escape is split symmetrically between two moving agents: both
      // bear half and veer apart in equal measure (reciprocal evasion). A
      // stationary partner cannot evade, so the moving agent bears the full
      // escape (the ORCA static-obstacle rule); otherwise a mover passing a
      // parked agent would under-escape by half and clip it.
      if (di_moving && dj_moving) {
        cons[i].push_back(
            Constraint{n, static_cast<float>(detail::dot2(velocities[i], n)) +
                              0.5f * half});
        cons[j].push_back(Constraint{n * -1.0f, static_cast<float>(detail::dot2(
                                                    velocities[j], n * -1.0f)) +
                                                    0.5f * half});
      } else if (di_moving) {
        cons[i].push_back(Constraint{
            n, static_cast<float>(detail::dot2(velocities[i], n)) + half});
      } else { // dj_moving
        cons[j].push_back(Constraint{
            n * -1.0f,
            static_cast<float>(detail::dot2(velocities[j], n * -1.0f)) + half});
      }
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (agents[i] && agents[i]->status == AgentStatus::Moving &&
        !cons[i].empty()) {
      desired[i] =
          constrained_velocity(desired[i], agents[i]->max_speed, cons[i]);
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
      const double d = static_cast<double>(length(to_target));
      // Direct safe landing: if the remaining distance can be covered within
      // this substep at the speed limit, move exactly onto the waypoint or
      // goal instead of scaling by `d / (speed * dt)`, whose float ratio
      // underflows to zero for tiny distances and huge but finite durations
      // and would leave a valid agent stuck one epsilon short of its goal.
      const bool reachable = d > 0.0 && d <= static_cast<double>(a.max_speed) *
                                                 static_cast<double>(dt);
      const float m = length(move);
      if (reachable) {
        move = to_target;
      } else if (d > 0.0 && m > static_cast<float>(d)) {
        move = move * (static_cast<float>(d) / m); // never pass the target
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
  // Per-pair escape state kept across substeps (see PairEscape).
  std::vector<PairEscape> pair_esc;
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
    advance(m_impl->agents, m_impl->mesh, dt, m_impl->pair_esc);
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
