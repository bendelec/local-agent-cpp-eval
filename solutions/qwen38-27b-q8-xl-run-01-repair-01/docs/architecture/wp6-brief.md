<!-- Working brief for the named work package; superseded by the final code and the
     revision log in implementation-plan.md. Kept as a historical record. -->
# WP6 Brief — Local avoidance (S6.2) + determinism soak (S6.3)

Extend `src/vwmini/simulation.cpp` (and `src/vwmini/simulation_impl.hpp` if
needed) with per-substep local avoidance, and add deterministic tests.
Public headers must NOT change. WP1–WP5 behavior must remain identical
(66 existing tests must stay green, warning-clean).

## Contract to satisfy (docs/requirements/simulation.md)

- SIM-010: every substep decides all velocities from a snapshot of all live
  positions/velocities before any update is applied. The current substep loop
  in `Simulation::step` already takes the snapshot in (a) and decides in (b) —
  avoidance is inserted into (b), reading only snapshot data (positions AND
  the previously decided snapshot velocities of others; do not read state
  mutated earlier in the same substep's (c)/(e) phases).
- SIM-011 (acceptance, deterministic tests required): in an open convex cell,
  two initially non-overlapping agents on crossing or overtaking routes:
  after every returned `step`, discs never overlap by more than `1e-3f` m;
  both agents eventually reach their goals (Reached).
- SIM-012: no teleporting (per-substep movement ≤ max_speed·dt plus the
  existing containment clamp), never exceed max_speed, state stays finite,
  agents stay inside the mesh.
- SIM-010 edge: initially overlapping agents (both centres contained) must
  remain finite and the implementation must attempt separation.

## Algorithm (binding skeleton; constants may be tuned)

In phase (b), after the waypoint steering computes `v` for agent `i`
(snapshot position `pos_i`, snapshot velocity of others = `decided` of prior
phase of THIS substep is NOT available — use `snap_vel`, the snapshot
velocities captured in (a); capture `snap_vel` alongside `snap_pos`):

```
for each other live agent j (slot order):
    rel  = pos_j - pos_i
    dist = |rel|
    r    = radius_i + radius_j
    // 1) already overlapping: push apart, anti-symmetric fallback when dist==0
    //    (smaller slot id pushes +x-ish, larger -x-ish: dir=(1,0) for i<j,
    //     (0,1)/(0,-1) is fine as long as deterministic and anti-symmetric)
    if dist < r:
        u = normalized(rel) if dist > 0 else fallback_dir(i, j)
        push += u * (max_speed_i * (r - dist) / r + 0.1 * max_speed_i) * (radius_i / r)
    // 2) predicted closest approach (orobools/RVO-lite)
    else:
        relv = v_j - v_i          // snapshot velocities
        q    = dot(rel, relv)
        if q < 0:                 // approaching
            t = -q / max(dot(relv, relv), 1e-12f)
            if t <= T_LOOK:       // T_LOOK ≈ 1.0..1.5 s
                dca = length(rel + relv * t)   // closest-approach distance
                if dca < r + MARGIN:           // MARGIN ≈ 0.5*(r_i+r_j) or 0.25 m
                    u = normalized(rel + relv * t)  // if zero-length: fallback_dir
                    push += u * min(max_speed_i, GAIN * (r + MARGIN - dca)) * (radius_i / r)
v += push
clamp |v| <= max_speed_i          // (existing clamp, now meaningful)
```

Guidelines:
- Keep it a flat function, e.g. `namespace { Vec2 avoid(SimAgent& self,
  const std::vector<SimAgent>& agents, const std::vector<Vec2>& snap_vel,
  float max_speed) }` or an equivalent; no allocation inside the substep loop
  beyond the vectors already used (you may reuse a single scratch buffer).
- All constants `constexpr` with names; document them.
- Idle/NoPath/Reached agents keep `v = {0,0}` (they are still obstacles in
  other agents' avoidance via the snapshot — do not change their own velocity).
- The landing-snap logic (dist <= max_speed·dt → exact waypoint landing)
  must remain consistent with avoidance: a landing agent may still be pushed;
  if pushed, it just does not land exactly this substep (its velocity keeps
  magnitude ≤ max_speed; landing is recomputed next substep). Make sure the
  landing path does not skip the avoidance push or exceed max_speed.
- Containment clamp (d) and post-move transitions (e) unchanged in spirit;
  the clamp must still guarantee containment after any pushed move.

## Tests (add to tests/simulation_test.cpp; deterministic)

Default fixture: 10×10 square (two triangles) or a single large triangle;
agents radius 0.25, max_speed 1.0–1.4. Drive scenarios with a fixed loop of
`step(0.1f)` (or a few `step` calls of varied finite durations); assert
invariants at every returned step; cap iterations (e.g. 1000) so a deadlock
fails the test rather than hangs.

1. **Crossing**: A at (1,5) goal (9,5); B at (5,1) goal (5,9); start
   `set_goal` for both, then step until both Reached. Assert at every step:
   for all pairs `dist >= r_i + r_j - 1e-3f`; finite positions; `contains`
   true; `|velocity| <= max_speed + 1e-6f`. Assert both end Reached within
   the cap.
2. **Overtaking**: A at (1,5) goal (9,5) max_speed 1.4; B at (2,5) goal (8,5)
   max_speed 1.0 (A starts behind). Same invariants; both Reached.
3. **Crossing at angle + speed mismatch** (variation of 1, e.g. A (1,1)→(9,9),
   B (1,9)→(9,1), speeds 1.0/1.4). Same invariants.
4. **Overlapping start** (SIM-010 edge): two agents radius 0.25 at (5,5) and
   (5.1,5) (both contained, discs overlap), goals apart (5,1)/(5,9); step for
   a bounded number of substeps: state finite, contained, and separation is
   attempted: `dist` strictly increases at some point / exceeds r_i+r_j-1e-3
   eventually, or at least does not decrease below the initial minus 1e-3.
   (Keep the assertion to robustness: finite + contained + attempted
   separation; do not require reaching goals.)
5. **No interference for far agents**: an idle agent far away does not change
   a moving agent's trajectory (compare positions step-by-step against a
   same-sim without the idle agent, first 20 steps — exact equality).
6. **Determinism soak (S6.3)**: build the scenario twice (fresh
   `NavMesh` + `Simulation`), same scripted calls (mixed step durations:
   0.1, 0.5, 0.33, 1.0, 0, plus set_goal mid-run), 500 substeps total; after
   each public call compare all agents' full `AgentState` (position,
   velocity, radius, max_speed, goal, status) for EXACT equality between the
   two runs. Use a 2-agent crossing + 1 idle agent scenario.

## Process

- Build + `ctest --test-dir build --output-on-failure` (all green,
  warning-clean `-Wall -Wextra -Wpedantic`).
- If invariants fail, tune constants (document values); do not weaken test
  assertions. If a genuine design flaw is found (e.g. oscillation that makes
  SIM-011 unachievable), change the approach within the skeleton and record
  it in the revision log.
- Update `docs/architecture/architecture.md` (avoidance module description
  must match the final code: snapshot semantics, algorithm name/sketch,
  constants) and `docs/architecture/implementation-plan.md` (S6.2, S6.3 done;
  revision entry: algorithm, constants chosen, test outcomes).
- Do not modify public headers; do not regress WP1–WP5 tests.
