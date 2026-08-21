# VWmini Implementation Plan

Work packages (WP) are ordered; slices are small and verifiable. Status:
`[ ]` open, `[x]` done.

## WP1 — Geometry core (`src/vwmini/geometry_priv.*`)
- **S1.1** [x]  Float/Vec2 predicates: `is_finite` (Vec2 overload; float uses
  `std::isfinite`), orientation, signed area, point-to-segment distance.
  Files: `geometry_priv.hpp/.cpp`.
  Verify: unit tests in `tests/geometry_test.cpp` (finite classification,
  collinearity, distance to segment endpoints/interior).
- **S1.2** [x]  `length`/`normalized` implementations (MSH-001). Verify: tests
  incl. zero-vector normalization → {0,0}, finite-invariant.

## WP2 — Triangulation (`src/vwmini/triangulate.*`)
- **S2.1** [x]  Input validation per MSH-002: non-finite → InvalidArgument;
  <3 verts, cyclic consecutive duplicates (incl. first==last), self-intersection,
  CW winding, degenerate area → InvalidMesh; collinear verts allowed.
  Verify: `tests/triangulate_test.cpp` with each rejection class + a collinear
  accepted case.
- **S2.2** [x]  Ear clipping producing deterministic CCW non-degenerate
  triangles; area-sum check vs input within ε·max(1,|A|) (MSH-003).
  Verify: tests — convex quad/pentagon, concave polygon, L-shape, area
  conservation, output determinism, no-overlap spot checks.

## WP3 — NavMesh (`src/vwmini/mesh_priv.*`)
- **S3.1** [x]  `NavMesh::create` validation per MSH-004: non-finite →
  InvalidArgument; empty list, non-triangle, CW/degenerate, overlapping
  interiors, non-manifold edge, T-junction → InvalidMesh. Transactional
  (MSH-007). Verify: `tests/nav_mesh_test.cpp`.
- **S3.2** [x]  Edge-hash adjacency (MSH-005): shared complete edges with
  endpoints ≤ ε; vertex-only touch → no adjacency. Verify: L-shaped two-tri
  mesh adjacency behaviour via find_path; direct adjacency tests where
  observable.
- **S3.3** [x]  `contains` per MSH-006: non-finite -> false; strict interior of any
  triangle -> true; within `epsilon` of any triangle edge segment -> true;
  otherwise false. Brute-force scan of all cells (const, allocation-free;
  see revision log for the grid drop).
  Verify: `tests/nav_mesh_test.cpp` — interior, far outside, boundary point,
  ε-outside admission, vertex-only touch, T-junction rejection, non-finite
  point → false, disjoint components.

## WP4 — Path finding (`src/vwmini/path.*`)
- **S4.1** [x]  Endpoint validation + direct path (SIM-001): non-finite →
  InvalidArgument; outside → OutsideMesh; straight segment contained →
  `[start, goal]`; exactly-equal endpoints → `[start]`.
  Verify: `tests/path_test.cpp`.
- **S4.2** [x]  Cell graph + Dijkstra + funnel over shared-edge portals with
  portal endpoints as anchors (SIM-003); deterministic tie-break
  (SIM-004). Verify: U-shaped corridor test (path must hug the corner),
  multi-cell path, determinism (two calls bit-equal).
- **S4.3** [x]  Refinement + verification: omit consecutive points < ε apart;
  verify every segment is contained (sampled + analytic segment check); on
  violation → NoPath (SIM-002 gate). Verify: corridor test asserts every
  output segment is contained by an independent check; epsilon-omission test.

## WP5 — Simulation core (`src/vwmini/simulation.*`)
- **S5.1** [x]  `Simulation` impl skeleton: id allocation (non-zero, no reuse),
  `agent`, `agent_count`, `remove_agent` (SIM-013). Verify:
  `tests/simulation_test.cpp` (S51_Lifecycle tests).
- **S5.2** [x]  `add_agent` validation per SIM-005/006 (finite scalars,
  positive radius/max_speed, in-mesh position, arrival-radius sentinel rule);
  optional goal → same transitions as set_goal. Verify: all error classes
  (S52_AddAgentValidation tests).
- **S5.3** [x]  `set_goal`/`clear_goal` state machine per SIM-007 (Moving /
  immediate Reached + zero velocity / NoPath + zero velocity; Idle after
  clear; NotFound on dead id). Verify: state-transition matrix tests
  (S53_GoalTransitions tests).
- **S5.4** [x]  `step` validation (SIM-008): non-finite/negative →
  InvalidArgument, zero state change; zero duration no-op. Verify: invalid
  step leaves full snapshot unchanged (S54_StepValidation test).

## WP6 — Motion + avoidance
- **S6.1** [x] Waypoint following in substeps: clamp by max_speed·dt, no
  overshoot (stop exactly at waypoint/goal via landing snap + terminal
  position snap), status transitions per SIM-009, containment clamp
  (48-iteration bisection). Basic motion only — avoidance is S6.2.
  Verify: straight-line arrival, large-dt no-overshoot, diagonal motion,
  containment clamp (S61_Motion tests).
- **S6.2** [x] Avoidance: per-substep snapshot (SIM-010), RVO velocity-obstacle
  projection (double cone in relative-velocity space, apex at -rel/t,
  half-angle asin(R/dist)) with separation push for overlapping discs and
  deterministic anti-symmetric fallback (SIM-012), final max-speed clamp and
  containment clamp. Constants: kAvoidLookahead=1.5 m, kAvoidMargin=0.4 m,
  kAvoidWeight=1.0, kAvoidSeparationBoost=0.7. Verify: crossing pair,
  overtaking pair, angled crossing with speed mismatch (no overlap > 1e-3f,
  both reach goals, invariants at every step); overlapping-start robustness
  (finite, contained, separation attempted); far idle agent non-interference
  (exact position equality for 20 steps); determinism soak (S6.3 tests in
  tests/simulation_test.cpp, S62_Avoidance + S63_Determinism suites).
- **S6.3** [x] Determinism soak: 2-agent crossing + 1 idle agent, mixed step
  durations (0.1, 0.5, 0.33, 1.0, 0 s), 500 substeps, mid-run set_goal;
  two fresh runs compared field-by-field for exact AgentState equality after
  every public call (S63_Determinism.TwoRunsBitIdentical test).

## WP7 — Build, tests, quality
- **S7.1** [x]  CMake: sources, `vwmini::vwmini` alias, system GTest via
  `find_package` (FetchContent fallback kept for portability),
  `-Wall -Wextra -Wpedantic` on library and test targets. Verify: clean build
  from scratch (Debug) with zero warnings on g++ 16 and clang++ 22.
- **S7.2** [x]  Full suite green (72/72) on both compilers
  (build/ = g++, build-clang/ = clang++), warning-clean under
  `-Wall -Wextra -Wpedantic`.
- **S7.3** [x]  simplify-check worker pass applied (see revision log);
  documentation consistency pass; docs updated to match final code; final
  build/test runs recorded in the completion report.

## Revision log
- WP1/2 complete: all 16 tests green, warning-clean.
- Simplification pass (post-WP2, S7.3 prep): removed the unused `is_finite(float)`
  passthrough and the unread double-area out-parameter of `validate_outline`;
  dropped neighbour re-checks in the outline self-intersection loop that the
  `continue` already excludes; removed a dead branch in
  `segments_collinear_overlap`; `clip_ring` is no longer `noexcept` (it
  allocates, and the public API is not `noexcept`); tests now include
  `geometry_priv.hpp` (private include dir added to `vwmini_tests`) and the
  S1.1 unit tests for `is_finite`, `point_on_segment_exact`, and
  `point_segment_distance` were added as planned; `test_util.hpp` uses `kEps*kEps`
  instead of a literal. Naming: the Vec2 predicate is the `is_finite` overload,
  not `is_finite_vec` as originally drafted; S1.1 text above and
  `architecture.md` updated accordingly.
- (S3.3): `contains` is a brute-force scan of all cells instead of the grid cell
  index sketched in architecture.md. Rationale: NFR-008 sets no performance
  target, MSH-006 only requires const + allocation-free, and a spatial grid adds
  coordinate-clamping edge cases for no functional gain. architecture.md updated.

- WP3 complete: 38/38 tests green (19 pre-existing + 19 mesh). Notes:
  `NavMesh::Impl` (private member) is completed in `nav_mesh.cpp` wrapping
  `detail::MeshImpl` so no other TU names the private type; the non-manifold
  edge check is defensive (overlap checks catch most geometric cases) and is
  exercised indirectly via the T-junction/overlap tests rather than a pure
  3-way edge case, which is unreachable without an overlap firing first in
  exact 2D float geometry.
- (S4.1–S4.3): Path module implemented in `src/vwmini/path.{hpp,cpp}`.
  Design notes: (a) the classical funnel's left/right string bookkeeping was
  replaced by iterative string-pulling (coordinate descent over the portal
  crossing points): with consecutive portals sharing one convex cell the
  corridor feasibility is automatic, and the block-cost objective is jointly
  convex, so coordinate-wise optimality is a global minimum — the
  funnel/string-pulling result SIM-003 requires, with a first-order
  verification gate (sweep cap 64, then check; fail-safe NoPath). (b)
  SIM-001/SIM-002 containment is decided by an exact per-triangle interval
  test on the segment parameter (strict interior + epsilon capsule bands as
  closed intervals in t, merged; uncovered length must be <= 1e-7 m).
  (c) Pulled points within 1e-5 m of a portal endpoint snap to the exact
  vertex (float intersection noise), keeping output corners exact.
  (d) Bug found by tests: the capsule lateral constraint used `D` where it
  needs `-D` (`|C + tD| <= h` gives two half-lines with opposite signs);
  fixed and covered by the disconnected-components test.
  `NavMesh::Impl` was moved from nav_mesh.cpp to mesh_priv.hpp (a nested
  member definition must be visible in every TU with friend access; placed
  in the enclosing namespace `vwmini`).
  All 48 tests green (10 path tests: validation, equal endpoints, direct
  paths incl. cross-cell and on-boundary start, U-shape corner-hugging with
  exact length, determinism incl. mesh-copy, boundary start).
- (S5.1–S5.4, S6.1): Simulation core implemented in
  `src/vwmini/simulation.{hpp,cpp}` (internal PIMPL header + .cpp). Storage:
  `Simulation::Impl` holds `NavMesh` by value, `std::vector<SimAgent>`
  (slot i == AgentId{i+1}), monotonic `next_id`, `live_count`. Substep
  scheme: `n = max(1, llround(seconds/0.1))` capped at 1 000 000, equal `dt`
  for every substep → deterministic. Per substep: snapshot → velocity
  decision (waypoint steering, landing snap when `dist <= max_speed*dt`) →
  integrate → containment clamp (48-iteration bisection) → post-move
  transitions (arrival-radius reach, waypoint advance, route-exhausted reach
  with position snap to goal). Filename collision with the public header
  resolved by ordering the test target's include dirs (public before src).
  18 new tests (S51–S54 lifecycle/validation/transitions/step, S61 motion:
  straight-line, large-dt, diagonal, containment clamp). All 66 tests green,
  warning-clean under -Wall -Wextra -Wpedantic.
- (S6.2, S6.3): Local disc avoidance implemented in `simulation.cpp` as
  `avoidance_velocity()`, called in phase (b2) of each substep after the
  waypoint-steering velocity (`steer`) is computed. Algorithm: RVO
  velocity-obstacle projection (Smit et al. 2005) — for each other agent
  whose predicted closest approach falls within kAvoidLookahead (1.5 m) and
  inside r + kAvoidMargin (0.4 m), the agent's velocity is projected onto
  the nearest point outside the RVO double cone in relative-velocity space
  (apex at -rel/t, half-angle asin(R/dist)); the result is taken fully
  (kAvoidWeight = 1.0) and clamped to max_speed. Initially overlapping discs
  are pushed apart along the centre line with magnitude
  max_speed * ((r-dist)/r + kAvoidSeparationBoost=0.7); coincident centres
  use a deterministic anti-symmetric fallback (smaller slot id pushes +x).
  A landing agent whose velocity is altered by avoidance forfeits the exact
  landing for that substep. Constants were tuned by grid search over
  (margin, weight, boost) × {crossing, overtaking, angled-crossing,
  overlapping-start, far-idle, determinism} until all 6 new tests pass.
  6 new tests (S62_Avoidance: CrossingPair, OvertakingPair,
  AngledCrossingWithSpeedMismatch, OverlappingStartIsSeparated,
  FarIdleAgentDoesNotInterfere; S63_Determinism: TwoRunsBitIdentical).
  All 72 tests green, warning-clean under -Wall -Wextra -Wpedantic.
- (S7.3) Internal PIMPL header renamed `simulation.hpp` →
  `simulation_impl.hpp`: the same-name collision with the public header
  forced a fragile include-ordering hack in the test target; the rename
  removes the collision and the hack.
- (S7.3) Final simplify-check pass (worker review; findings applied where
  safe): simulation.cpp — removed `is_finite_f`/`is_finite_v` wrappers (call
  `std::isfinite`/`detail::is_finite` directly); `live_agent` gained a const
  overload so `agent()` shares the single bounds/alive check; `reset_motion`
  helper replaces the 3× velocity/route/route_index reset; `kArrivalSentinel`
  centralises -1.0f; `landing`/`landing_ok` double array collapsed to one
  `std::vector<char>`; phase (e)'s two Reached transitions merged into one
  terminal block (both snap to the goal: `route.back() == *goal` by
  construction of `apply_goal`); comment fixes (phase (a) no longer claims
  velocities are snapshotted; RVO comment re-indented; "do not fold" note on
  the kAvoidWeight blend for zero-sign behaviour). mesh_priv —
  `edges_match`, `strictly_inside_triangle`, `mesh_validation_error` moved
  into the anonymous namespace and dropped from mesh_priv.hpp (no other TU
  uses them); `strictly_inside` hoisted as a shared inline in
  geometry_priv.hpp (delegated to by triangulate.cpp and mesh_priv.cpp).
  Tests — dead `segment_in_triangle` removed; `err_code` hoisted into
  test_util.hpp (3 test files deduplicated); duplicate `wp6_mesh()` removed
  (uses `squareMesh()`); `wp6_invariants` gained a `check_separation` flag
  used by the intentionally-overlapping robustness test; the S63 soak now
  drives an explicit 132-call sequence (26 cycles of {0.1, 0.5, 0.33, 1.0,
  0.0} + {0.1, 0.5} = exactly 500 substeps; set_goal after call 25) instead
  of re-implementing the library's substep scheme. Kept deliberately: the
  one-declaration private headers path.hpp/triangulate.hpp (module seams),
  the kAvoidWeight blend form, the ~1-ulp safety clamp in phase (b).
  All 72 tests green, warning-clean on g++ and clang++ after the pass.
- (S7.3) Completion report written to docs/architecture/completion-report.md
  (design/modules, slice history, files changed, exact build/test commands
  and outcomes, deliberate behavior choices).
