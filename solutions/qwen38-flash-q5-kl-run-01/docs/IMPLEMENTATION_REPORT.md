# VWmini — Implementation Report

## 1. Design and modules (and why)

Layered, cycle-free, no frameworks or inheritance. Public headers stay exactly as supplied
(only one private addition: `friend Result<Path> find_path(...)` in `nav_mesh.hpp`).

| Module | Responsibility |
|---|---|
| `src/predicates.hpp` | Leaf, header-only geometry primitives + `epsilon` + finiteness helpers. Nothing above it re-derives a tolerance. |
| `src/geometry.cpp` | Public `length`, `normalized` (MSH-001). |
| `src/triangulation.{hpp,cpp}` | Outline validation + ear clipping (MSH-002/003). |
| `src/mesh_topology.{hpp,cpp}` | The one validated triangle set: acceptance validation (MSH-004), adjacency (MSH-005), `contains` (MSH-006), `locate`, `shared_edge`, `segment_is_contained`. |
| `src/nav_mesh_impl.hpp` / `nav_mesh.cpp` | `NavMesh::Impl` seam (`shared_ptr<const Impl>`) and the thin public façade. |
| `src/corridor.{hpp,cpp}` | Deterministic Dijkstra over cell adjacency (SIM-004). |
| `src/pathfinding.cpp` | `find_path`: endpoint validation, direct-line shortcut, portal-seeded string pulling, output simplification (SIM-001/002/003). |
| `src/agent.hpp` | `AgentRuntime`: one agent's state + the transitions that depend only on it (`adopt_route`, `clear_goal`, `target`, `refresh_arrival`). |
| `src/steering.{hpp,cpp}` | Local avoidance: one agent's velocity for one substep from a snapshot (SIM-010…012). |
| `src/simulation.cpp` | `Simulation::Impl`: agent table + id recycling, all public validation, `route_to_goal`, substepped stepping. |
| `src/error.hpp` | The only place a public `Error` is built. |

Key trade-offs (documented in `docs/architecture/architecture.md` §6): triangles are stored as
submitted (no welding → T-junction rejection stays exact and `cell_count` is free); segment
containment by critical-parameter sampling over the cells a segment touches; **verified greedy
string pulling** instead of a hand-written funnel; **collision-cone deflection** instead of an
ORCA/VOR+ solver; containment-safe integration rather than post-hoc pushback; free-slot id
recycling so `agent_count` stays exact and indices (not pointers) drive determinism.

## 2. Work packages / slices

`docs/architecture/implementation-plan.md` holds WP0…WP7 with per-slice goal, dependencies,
files, verification and status. **All slices are `done`.** Explicit revisions recorded in its
revisions log (each with the evidence that forced the change):

1. `locate` returns *all* containing cells (boundary points belong to several).
2. Dropped the planned `src/simulation_impl.hpp`; `Impl` lives in `simulation.cpp`.
3. Validation order inside `MeshTopology::build` → cells, adjacency/non-manifold, T-junction,
   interpenetration (a non-manifold triple necessarily also overlaps).
4. `funnel` module replaced by verified greedy shortening; later revised again — the pull is
   seeded with **portal endpoints**, not cell centroids (a centroid seed stalls on corners).
5. Planned "forward cone" filter and ordered candidate scan **not** implemented (the swept-segment
   test already ignores separating/out-of-horizon pairs); there is no braking rule.
6. `agent_motion.{hpp,cpp}` folded into `agent.hpp` + `advance_agent`.
7. **Bug found in review:** waypoint capture was inverted (`dot(pos-waypoint, prev-waypoint) >= 0`
   is true while still approaching), so whole routes were discarded and an agent walked into a wall
   instead of rounding a corner. Fixed by projecting against `previous → target`, and the test gap
   that hid it (no fixture had a turn) was closed.
8. Final coverage gaps closed: overtaking acceptance (SIM-011), shared `expect_physical_invariants`
   (containment + speed cap every multi-agent step), `PathTest.VertexOnlyTouchCreatesNoAdjacency`
   (MSH-005); verification claims reworded to describe what is actually asserted.

## 3. Exact commands and outcomes

```sh
cmake -S . -B build                                  # ok (fresh configure)
cmake --build build --parallel                       # ok, 0 warnings (-Wall -Wextra -Wpedantic
                                                     #  plus -Wshadow -Wconversion -Wsign-conversion
                                                     #  -Wold-style-cast -Wcast-align …)
ctest --test-dir build --output-on-failure           # 100% tests passed out of 69
clang-format --dry-run -Werror $(find src tests include -name '*.hpp' -o -name '*.cpp')   # clean
bash scripts/check.sh build                          # ALL CHECKS PASSED (the whole gate above)

cmake -S tests/consumer -B build/consumer -DVWMINI_SOURCE_DIR=$PWD -DVWMINI_BUILD_TESTS=OFF
cmake --build build/consumer --parallel
./build/consumer/consumer                            # exit 0 — links vwmini::vwmini as a downstream user
```

Test suites: `geometry_test` (7), `triangulation_test` (12), `nav_mesh_test` (17),
`path_test` (10), `simulation_test` (23) = 69, all deterministic (fixed inputs, fixed iteration
order, no time/randomness/containers keyed by floats).

## 4. Files

* Added: `src/{predicates.hpp, geometry.cpp, triangulation.{hpp,cpp}, mesh_topology.{hpp,cpp},
  nav_mesh_impl.hpp, nav_mesh.cpp, corridor.{hpp,cpp}, pathfinding.cpp, agent.hpp,
  steering.{hpp,cpp}, simulation.cpp, error.hpp}`, `tests/{mesh_fixtures.hpp, geometry_test.cpp,
  triangulation_test.cpp, nav_mesh_test.cpp, path_test.cpp, simulation_test.cpp}`,
  `tests/consumer/{CMakeLists.txt, main.cpp, README.md}`, `scripts/{check.sh, fmt.sh}`,
  `.clang-format`, `docs/architecture/{architecture.md, implementation-plan.md}`,
  `docs/IMPLEMENTATION_REPORT.md`.
* Modified: `CMakeLists.txt` (library sources, strict warnings, alias `vwmini::vwmini`,
  C++23, `VWMINI_BUILD_TESTS`), `tests/CMakeLists.txt`, `README.md` (rewritten as an accurate
  library README), `include/vwmini/nav_mesh.hpp` (private `friend` declaration only).
* Deleted: placeholder `src/vwmini.cpp`.
* Requirements, public API doc and public declarations: unchanged.

## 5. Contract points where more than one implementation is legal — my choices

* **Route shortening**: verified greedy "jump to furthest visible waypoint" over a
  portal-seeded polyline; taut but not proven optimal (SIM-003/SIM-004 allow this).
* **Winner among equal-cost routes**: not prescribed — Dijkstra ties break on cell index.
* **Segment containment**: exact convex-interval intersection glued at shared portals plus
  endpoint bridging with an explicit containment re-check; no unproven bridging.
* **Avoidance**: collision-cone deflection, deterministic side choice, ±60°/±120° fallback
  headings, positional separation for initially overlapping discs (SIM-010 edge case), fixed
  axis for exactly concentric discs; no braking, only a max-speed clamp.
* **Substepping**: `min(seconds / kPreferredSubstep + 1, kMaxSubsteps)` clamped in float —
  a pure function of the argument; large durations cannot overshoot a waypoint or goal.
* **`agent_count` exactness**: free-slot recycling with generation-stamped ids, so removed ids
  keep returning `NotFound` while indices stay the deterministic iteration order.
* **Tolerances**: contract `epsilon` reused for the two route-state tolerances that measure the
  same scale; every other tolerance is its own named constant.
