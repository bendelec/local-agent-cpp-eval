<!-- Working brief for the named work package; superseded by the final code and the
     revision log in implementation-plan.md. Kept as a historical record. -->
# WP5 Brief — Simulation core (S5.1–S5.4 + S6.1 basic motion)

Implement the `vwmini::Simulation` public API (fixed header
`include/vwmini/simulation.hpp` — do NOT change it) in a new internal header
`src/vwmini/simulation.hpp` + `src/vwmini/simulation.cpp`. Add
`src/vwmini/simulation.cpp` to the `vwmini` target in `CMakeLists.txt`.
Deterministic tests in `tests/simulation_test.cpp` (see `tests/CMakeLists.txt`
for the existing pattern; GTest is available).

This brief is the binding design contract. Where it is silent, follow
`docs/architecture/architecture.md` and the requirements.

## Storage

PIMPL as the public header prescribes: `Simulation::Impl` (defined in the
internal header) holds:

```cpp
struct Agent {
    bool alive{false};
    Vec2 position{}, velocity{};
    float radius{}, max_speed{};
    std::optional<Vec2> goal;
    float arrival_radius{};        // effective (>= 0), resolved at goal time
    AgentStatus status{AgentStatus::Idle};
    std::vector<Vec2> route;       // waypoints, excludes the start point
    std::size_t route_index{0};    // index of the current target waypoint
};
// Impl:
NavMesh mesh;                       // by value (shared_ptr<const Impl> inside)
std::vector<Agent> agents;          // slot i  ==  AgentId{ i+1 }
std::uint32_t next_id{1};           // monotonic, no reuse after removal
std::size_t live_count{0};
```

Public `Simulation(NavMesh mesh)` (takes by value — public header) constructs
`m_impl` from it. Move ctor/assign: default (unique_ptr moves). Dtor default.

## add_agent (SIM-005, SIM-006)

Validation order, first failure wins:

1. Non-finite `position`, `goal` (if present), `radius`, `max_speed`,
   `arrival_radius` → `InvalidArgument`.
2. `radius <= 0` or `max_speed <= 0` → `InvalidArgument`.
3. `arrival_radius < 0 && arrival_radius != -1.0f` → `InvalidArgument`.
   (`-1.0f` is the sentinel; `-0.0f` is valid and means 0.)
4. `!mesh.contains(position)` → `OutsideMesh`.
5. If goal present and `!mesh.contains(goal)` → `OutsideMesh`.

Then append an `alive` agent at slot `next_id`, `++next_id`,
`++live_count`, and if a goal is present run the goal transition below (any
`OutsideMesh`/validation error is already excluded; a disconnected goal is
not an error). Return the id.

## Goal transition (shared by add_agent and set_goal)

```
effective = (arrival_radius == -1.0f) ? agent.radius : arrival_radius
```

- If `distance(position, goal) <= effective`: `status = Reached`,
  `velocity = {0,0}`, `goal = goal`, `route = {}`, `route_index = 0`.
- Else compute `find_path(mesh, position, goal)`:
  - `NoPath` error → success: `status = NoPath`, `velocity = {0,0}`,
    `goal = goal`, `route = {}`.
  - Success → `status = Moving`, `goal = goal`,
    `route = path.points` with the first point (== position) dropped,
    `route_index = 0` (target = first real waypoint; route's last point is
    the goal).

## set_goal (SIM-007)

1. Live id? else `NotFound`. (id.value == 0 is unknown.)
2. `!is_finite(goal)` → `InvalidArgument`.
3. Arrival-radius sentinel rule → `InvalidArgument`.
4. `!mesh.contains(goal)` → `OutsideMesh`.
5. Apply the goal transition (replace previous goal/route/status).

## clear_goal (SIM-007)

Live id? else `NotFound`. Then `goal = nullopt`, `route = {}`,
`route_index = 0`, `velocity = {0,0}`, `status = Idle`.

## remove_agent (SIM-013)

Live id? else `NotFound`. Mark slot `alive = false`, `--live_count`. Id is
never reused.

## agent / agent_count (SIM-013)

`agent(id)`: `id.value == 0` or out of range or `!alive` → `std::nullopt`;
else value-copy `AgentState {position, velocity, radius, max_speed, goal,
status}`. `agent_count()` → `live_count`. Neither mutates state.

## step (SIM-008, SIM-009)

1. `!std::isfinite(seconds) || seconds < 0.0f` → `InvalidArgument`, no change.
2. `seconds == 0.0f` → success, no change.
3. Substeps: `n = max(1, llround(seconds / 0.1f))` capped at 1'000'000;
   `dt = seconds / (float)n` (same dt for every substep → deterministic).
4. Per substep, in this order:
   a. **Snapshot** (SIM-010): copy positions+velocities of all live agents in
      slot order into plain vectors. All velocity decisions in (b) read only
      this snapshot.
   b. Decide each live agent's substep velocity (slot order):
      - `status != Moving` → `{0,0}`.
      - `Moving`: `target = route[route_index]`, `to = target - pos`,
        `dist = length(to)`. If `dist <= max_speed * dt` → `v = to / dt`
        (lands exactly on the waypoint, no overshoot); else
        `v = normalized(to) * max_speed`. (Avoidance hook for WP6 comes
        here; WP5 keeps `v` as-is.) Clamp `length(v) <= max_speed`.
   c. Integrate all: `pos += v * dt` (using the snapshot-derived decisions).
      Store the decided `v` as the agent's velocity.
   d. **Containment clamp** (SIM-012): if `!mesh.contains(new_pos)`,
      bisect `t` in `[0,1]` (48 iterations; `t=0` is contained because the
      previous position was contained) for the largest contained point
      `pos + t*(new_pos - pos)`; set that as the new position.
   e. Post-move transitions:
      - If `goal` present and `length(goal - pos) <= arrival_radius` (the
        effective radius stored on the agent) → `status = Reached`,
        `velocity = {0,0}`.
      - Else while `route_index < route.size()` and
        `length(route[route_index] - pos) <= max(1e-4f, max_speed * dt)`
        → `++route_index` (advance past passed waypoints).
      - Else if `route_index >= route.size()` → `status = Reached`,
        `velocity = {0,0}` (route exhausted ⇒ at goal; covers
        `arrival_radius == 0`).

Error `message` strings: short, stable, human-readable.

## Tests (tests/simulation_test.cpp, deterministic)

Reuse `tests/test_util.hpp` helpers where they fit. Square mesh
`[{0,0},{10,0},{0,10}]` (two triangles) as the default fixture; build meshes
with `NavMesh::create`.

- S5.1: add → non-zero id; distinct ids; `agent()` snapshot equals config
  (Idle, zero velocity, goal empty); `agent_count()` exact; remove → id dead
  (`NotFound` from set_goal/clear_goal/remove, empty `agent()`); id 0 and a
  bogus large id → NotFound/empty; removed id is never reused (next add gets
  a larger value).
- S5.2 add_agent matrix: non-finite position (nan & inf), non-finite radius,
  radius 0, negative radius, max_speed 0, arrival_radius -2 / -0.5 / -1.5 →
  InvalidArgument (mesh/agent_count unchanged); position outside mesh →
  OutsideMesh; goal outside mesh → OutsideMesh; `-0.0f` arrival is valid.
- S5.3 transitions: far goal → Moving; goal within arrival radius → Reached
  + zero velocity + goal stored; disconnected two-triangle mesh, goal in the
  other component → success with NoPath + zero velocity; clear_goal → Idle +
  empty goal + zero velocity, subsequent step leaves position unchanged;
  set_goal on unknown/removed id → NotFound; set_goal(nan) → InvalidArgument;
  set_goal(arrival -0.5) → InvalidArgument.
- S5.4: step(-1), step(nan), step(inf) → InvalidArgument and the full agent
  snapshot (pos/vel/status/goal) unchanged; step(0) → success, unchanged.
- S6.1 motion: agent (1,1), goal (3,1), radius 0.1 (so default arrival 0.1
  doesn't pre-trigger), max_speed 1. `step(1.0f)` → position exactly (2,1),
  Moving. `step(1.0f)` → position exactly (3,1), Reached, velocity (0,0).
  Overshoot: fresh agent, `step(100.0f)` → position exactly (3,1), Reached,
  never past the goal.

## Process

- Build: `cmake --build build --parallel`; tests:
  `ctest --test-dir build --output-on-failure`.
- Warning-clean under `-Wall -Wextra -Wpedantic` (already on the target).
- Update `docs/architecture/implementation-plan.md`: mark S5.1–S5.4 and S6.1
  statuses honestly (S6.1 done = basic waypoint motion; note avoidance is
  still S6.2) and add a revision-log entry describing the implementation
  (storage shape, substep scheme, containment clamp, test count).
- Do not touch WP1–WP4 code except: none. Do not change any public header.
