# Implementation Plan

Ordered work packages (WP) and small verifiable slices. Status: `todo` / `doing` /
`done` / `revised`. Verification is always the same three-part gate: build
(`cmake --build build --parallel`), run tests (`ctest --test-dir build --output-on-failure`),
and no new warnings under `-Wall -Wextra -Wpedantic`.

Global gate after every WP: simplify/quality pass over the files just touched.

## WP0 — Build skeleton

| Slice | Goal | Depends on | Files / interfaces | Verification | Status |
|---|---|---|---|---|---|
| 0.1 | Library target compiles with C++23, tests wired to GTest/CTest, warnings on | — | `CMakeLists.txt`, `tests/CMakeLists.txt`, `tests/geometry_test.cpp` (smoke: target links and runs) | configure + build + ctest green | done |
| 0.2 | Delete placeholder `src/vwmini.cpp`, list real sources | 0.1 | `CMakeLists.txt`, delete `src/vwmini.cpp` | build green | done |

## WP1 — Vector + predicates (MSH-001)

| Slice | Goal | Depends on | Files / interfaces | Verification | Status |
|---|---|---|---|---|---|
| 1.1 | `length`, `normalized` (`{0,0}` → `{0,0}`) | 0.1 | `src/geometry.cpp` | `tests/geometry_test.cpp`: magnitudes, zero vector, huge/tiny magnitudes | done |
| 1.2 | Internal predicates: signed area, strict point-in-triangle, point-segment distance, segment intersection, critical-parameter segment sampling | 1.1 | `src/predicates.hpp` | no dedicated predicate tests: predicates are observable only through the containment, path and steering suites, which cover them (deliberate) | done |

## WP2 — Outline triangulation (MSH-002, MSH-003)

| Slice | Goal | Depends on | Files / interfaces | Verification | Status |
|---|---|---|---|---|---|
| 2.1 | Finiteness of all outline vertices → `InvalidArgument` | 1.2 | `src/triangulation.{hpp,cpp}` | tests | done |
| 2.2 | Outline validation: <3 vertices, cyclic duplicates, CW, degenerate area, self-intersection → `InvalidMesh`; collinear vertex allowed | 1.2 | `src/triangulation.cpp` | `tests/triangulation_test.cpp` | done |
| 2.3 | Ear clipping producing CCW non-degenerate triangles, area preserved within `epsilon*max(1,area)`, deterministic order | 1.3 | `src/triangulation.cpp` | area sum + CCW + non-overlap + repeat-determinism tests | done |

## WP3 — Mesh topology, containment, validation (MSH-004…MSH-007)

| Slice | Goal | Depends on | Files / interfaces | Verification | Status |
|---|---|---|---|---|---|
| 3.1 | `MeshTopology` value type: cells + neighbour lists; build from triangles | 1.2 | `src/mesh_topology.{hpp,cpp}` | `cell_count` assertions; adjacency is unobservable except through routes, so it is asserted there | done |
| 3.2 | Edge matching (MSH-005) + non-manifold detection | 3.1 | `mesh_topology.cpp` | creation tests for the 3-way edge/T-junction, plus `PathTest.VertexOnlyTouchCreatesNoAdjacency` for vertex-only touch | done |
| 3.3 | Rejection rules: empty list, non-triangle, CW/degenerate, interior overlap, T-junction, non-finite | 3.1 | `mesh_topology.cpp` | `tests/nav_mesh_test.cpp` per-case error codes | done |
| 3.4 | `contains` (MSH-006) + `locate` + `segment_is_contained` | 3.1 | `mesh_topology.cpp` | boundary-band, outside, non-finite, disjoint components tests | done |
| 3.5 | `NavMesh::create/contains/cell_count` façade + `Impl` | 3.2,3.4 | `src/nav_mesh_impl.hpp`, `src/nav_mesh.cpp` | MSH-007 transactional + copy-value tests | done |

## WP4 — Pathfinding (SIM-001…SIM-004)

| Slice | Goal | Depends on | Files / interfaces | Verification | Status |
|---|---|---|---|---|---|
| 4.1 | Dijkstra corridor over adjacency, deterministic ties | 3.5 | `src/corridor.{hpp,cpp}` | observed through `find_path`: disconnected components → `NoPath`, vertex-only touch → `NoPath` | done |
| 4.2 | Shorten the corridor: greedy furthest-visible string pulling (verified contained) | 4.1 | `src/pathfinding.cpp` | `tests/path_test.cpp`: no redundant waypoints, every segment contained, L-shape corner hugging | done |
| 4.3 | `find_path` glue: validation, direct shortcut, `< epsilon` point removal, endpoint pinning, `NoPath` | 4.2 | `src/pathfinding.cpp` | `tests/path_test.cpp`: error codes, `[start]`, `[start,goal]`, L-shape corner, containment sampling of every segment, determinism | done |

## WP5 — Agents and stepping (SIM-005…SIM-009, SIM-013)

| Slice | Goal | Depends on | Files / interfaces | Verification | Status |
|---|---|---|---|---|---|
| 5.1 | `Simulation::Impl`, agent table, monotonic ids, `add_agent`/`remove_agent`/`agent`/`agent_count` | 3.5 | `src/simulation.cpp`, `src/agent.hpp` | id validity, `NotFound` after removal, snapshots by value | done |
| 5.2 | Config validation + arrival-radius sentinel (SIM-006) + goal transitions | 5.1, 4.3 | `src/agent.hpp`, `simulation.cpp` | all SIM-005/006/007 cases incl. `-0.0f`, `-1.0f`, `-2.0f` | done |
| 5.3 | Route following + substepping + containment-safe placement + arrival (SIM-008/009) | 5.2 | `src/agent.hpp` (state + transitions), `src/simulation.cpp` (`advance_agent`) | travel-to-goal, large `step` no overshoot, invalid `step` is transactional, mesh-constrained | done |
| 5.4 | `clear_goal`, move semantics, no-move-when-idle | 5.3 | `simulation.cpp` | SIM-007 idle behaviour tests | done |

## WP6 — Local avoidance (SIM-010…SIM-012)

| Slice | Goal | Depends on | Files / interfaces | Verification | Status |
|---|---|---|---|---|---|
| 6.1 | Snapshot velocity decision: predicted-meeting screen over the swept step, collision-cone deflection to a free heading, positional separation for already-overlapping discs, max-speed clamp | 5.3 | `src/steering.{hpp,cpp}` | observed through the two-/three-agent acceptance tests in `tests/simulation_test.cpp` (there is no separate steering unit suite: the rules only matter as behaviour) | done |
| 6.2 | Wire into substep; speed/containment invariants | 6.1 | `simulation.cpp` | crossing + overtaking acceptance (SIM-011), max-speed and containment invariants, overlap ≤ 1e-3, overlapping-start robustness | done |
| 6.3 | Determinism of multi-agent runs | 6.2 | tests | identical runs produce identical agent states (full snapshot equality) | done |

## WP7 — Consolidation

| Slice | Goal | Depends on | Files / interfaces | Verification | Status |
|---|---|---|---|---|---|
| 7.1 | Warning-clean `-Wall -Wextra -Wpedantic` (+ `-Wshadow -Wconversion` review), clang-format pass | all | all | clean build log, `clang-format --dry-run -Werror` | done |
| 7.2 | Review + simplify pass; remove dead code/magic numbers | all | `src/`, `tests/` | review findings closed | done |
| 7.3 | Documentation consistency pass (docs match code) | all | `docs/architecture/*.md` | doc-check worker pass | done |
| 7.4 | Consumer smoke check: `find_package`-free link of `vwmini::vwmini` from a scratch project | all | `tests/consumer/` | external build compiles + runs | done |

## Revisions log

* (initial) Slices numbered per area; WP4 split into corridor / funnel / glue because the
  three concerns are independently testable.
* (after WP3) `mesh_topology` exposes `locate` returning *all* containing cells, not one
  cell: boundary points legitimately belong to several cells, and picking one would make
  corridor search needlessly fragile.
* (after WP3) `NavMesh::cell_count` must report the *accepted input triangle count*;
  since the topology stores triangles unchanged (design decision 1) these are the same
  number, so no extra bookkeeping was added.
* (after WP5) plan called for `src/simulation_impl.hpp`; evidence: only `simulation.cpp`
  needs `Simulation::Impl`, so it is defined there and the extra header was dropped.
  Steering receives plain agent views instead of simulation internals.
* (during slice 6.1) the planned "forward cone" filter was **not** implemented. Evidence:
  `meets_within_horizon` measures the closest point of the *swept* relative segment, so pairs
  that are moving apart have their nearest approach behind them and pairs more than
  `kLookAhead` away are ignored; a leader therefore never reacts to a follower behind it. An
  extra half-plane filter would have been a second, redundant rule. There is no braking rule:
  speed is only capped at `max_speed`. `architecture.md` §6 records the rules actually used.
* (during slice 6.1) `steering` chooses a velocity by rotating the relative motion onto the
  nearer collision-cone edge rather than by the planned ordered candidate scan: same
  determinism, fewer arbitrary constants, and the deflection is geometrically minimal.
* (during slice 5.3/6.2) `agent_motion.{hpp,cpp}` was folded into `agent.hpp` (state plus
  its transitions) and `advance_agent()` in `simulation.cpp`. Evidence: the motion rules need the
  route state and nothing else, and splitting them across two files spread one decision over
  two places. `architecture.md` §2 reflects the merged module.
* (after WP3) Validation order inside `MeshTopology::build` changed to
  cells → adjacency/non-manifold → T-junction → interpenetration. Evidence: three cells
  sharing one complete edge necessarily also overlap, so running adjacency first reports
  the more precise rule first (the public code is `InvalidMesh` either way).
* (before WP4) **`funnel` module replaced by verified greedy shortening.** Slice 4.2 is
  implemented as `pathfinding`'s "jump to the furthest mesh-verified visible waypoint" pass:
  every retained segment is
  explicitly checked with `MeshTopology::segment_is_contained`, which makes SIM-002's
  containment guarantee true by verification rather than by a funnel proof, and removes ~120
  lines of fiddly geometry. Trade-off: routes are taut but not proven optimal, which
  MSH-008/SIM-004 explicitly leave open. Slice 4.2 verification is therefore "no redundant
  waypoints + every segment contained + shorter than the centroid route" instead of funnel
  portal tests. (Revised again during review: the seed is portal endpoints, not cell
  centroids — a centroid seed stalls on corners; see the WP5 revision below.)

* (during slice 5.3, found in review) the waypoint-capture rule was wrong: a waypoint was
  dropped when `dot(position - waypoint, previous - waypoint) >= 0`, which is true while the
  agent is still *approaching* it, so whole routes were discarded and an agent steered straight
  into a wall instead of rounding the corner. Evidence: an L-corridor run wedged at the corner
  forever while every test still passed — no fixture had a turn. Fixed by testing the projection
  of `position` onto `previous → target` (`advance_agent` in `simulation.cpp`), and by adding
  corner fixtures plus `SimulationTest.AgentRoutesAroundACornerAndArrives` and
  `CoarseStepsStillRoundTheCorner`. Lesson recorded: waypoint capture is a route-state rule and
  needs a mesh with a turn in the test set.
* (during final review) coverage gaps closed: an overtaking acceptance case
  (`OvertakingAgentsPassWithoutOverlapping`, SIM-011), a shared `expect_physical_invariants`
  helper asserting containment and the speed cap during every multi-agent step, and
  `PathTest.VertexOnlyTouchCreatesNoAdjacency` (MSH-005). Verification claims in the tables above
  were corrected to describe what is actually asserted, because adjacency and predicates are only
  observable through routes and containment.

Statuses are `todo` / `doing` / `done` / `revised` (as in the header). **All slices in every work package are
`done`**; the final gate ran `scripts/check.sh` (clean configure, build, `ctest`,
`-Wall -Wextra -Wpedantic`, `clang-format --dry-run -Werror`) plus the consumer smoke
check in `tests/consumer/`. The revisions log above is the running record of the decisions
evidence changed.
