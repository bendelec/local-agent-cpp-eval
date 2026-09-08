# VWmini — Implementation Report

## 1. Design and modules (and why)

Layered, cycle-free, no frameworks or inheritance. Public headers stay exactly as supplied
(only one private addition: `friend Result<Path> find_path(...)` in `nav_mesh.hpp`).

| Module | Responsibility |
|---|---|
| `src/predicates.hpp` | The numeric policy in one place: `epsilon`, finiteness helpers and every geometry decision helper (signed area, containment, segment distance/projection, intersection parameter, crossing) computed on **double** intermediates. Nothing above it re-derives a tolerance or re-does arithmetic in `float`. |
| `src/vec2d.hpp` | `Vec2d`, the double working vector implicitly built from a `Vec2` (float→double conversion is exact). Storage and public types stay `Vec2`. |
| `src/geometry.cpp` | Public `length`, `normalized` (MSH-001). |
| `src/triangulation.{hpp,cpp}` | Outline validation + ear clipping (MSH-002/003). |
| `src/mesh_topology.{hpp,cpp}` | The one validated triangle set: acceptance validation (MSH-004), adjacency (MSH-005), `contains` (MSH-006), `locate`, `shared_edge`, `segment_is_contained`. |
| `src/nav_mesh_impl.hpp` / `nav_mesh.cpp` | `NavMesh::Impl` seam (`shared_ptr<const Impl>`) and the thin public façade. |
| `src/corridor.{hpp,cpp}` | Deterministic Dijkstra over cell adjacency (SIM-004). |
| `src/pathfinding.cpp` | `find_path`: endpoint validation, direct-line shortcut, portal-seeded string pulling, output simplification (SIM-001/002/003). |
| `src/agent.hpp` | `AgentRuntime`: one agent's state + the transitions that depend only on it (`adopt_route`, `clear_goal`, `target`, `refresh_arrival`). |
| `src/steering.{hpp,cpp}` | Local avoidance: one agent's velocity for one substep from a snapshot (SIM-010…012). |
| `src/simulation.cpp` | `Simulation::Impl`: agent table with monotonic ids, all public validation, `route_to_goal`, substepped stepping. |
| `src/error.hpp` | The only place a public `Error` is built. |

Key trade-offs (documented in `docs/architecture/architecture.md` §7; the numeric policy is §6): triangles are stored as
submitted (no welding → T-junction rejection stays exact and `cell_count` is free); segment
containment by critical-parameter sampling over the cells a segment touches; **verified greedy
string pulling** instead of a hand-written funnel; **collision-cone deflection** instead of an
ORCA/VOR+ solver; containment-safe integration rather than post-hoc pushback; a table of agents
with monotonic ids so `agent_count` is the table size and indices (not pointers) drive determinism.

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
   instead of rounding a corner. Fixed by projecting against the motion actually taken
   (`previous → position`), and the test gap
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
ctest --test-dir build --output-on-failure           # 100% tests passed out of 78
clang-format --dry-run -Werror $(find src tests include -name '*.hpp' -o -name '*.cpp')   # clean
bash scripts/check.sh build                          # ALL CHECKS PASSED (the whole gate above)
```

The same tree, built and run in the configurations that matter for this change:

| Configuration | Command | Outcome |
|---|---|---|
| GCC 16.2.1, Debug | `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug` | 0 warnings, 78/78 pass |
| GCC 16.2.1, Release (`NDEBUG`) | `cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release` | 0 warnings, 78/78 pass — the numeric regressions do not depend on `assert` |
| Clang 22.1.8, Debug | `cmake -S . -B build-clang -DCMAKE_CXX_COMPILER=clang++` | 0 warnings (incl. `-Wdouble-promotion`), 78/78 pass |
| ASan + UBSan | `-DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer"` | 0 warnings, 78/78 pass, no sanitizer reports |
| Library only | `cmake -S . -B build-notests -DBUILD_TESTING=OFF` | configures and builds `libvwmini.a`, no test targets |
| Alias option | `cmake -S . -B build-alias -DVWMINI_BUILD_TESTS=OFF` | same: `BUILD_TESTING` is the driver, the alias forwards to it |
| Downstream consumer | `cmake -S tests/consumer -B build/consumer -DVWMINI_SOURCE_DIR=$PWD -DVWMINI_BUILD_TESTS=OFF` | exit 0 (includes the extreme-coordinate checks below) |

Test suites, by registered suite name: `Vec2Test` (7), `NumericsTest` (9), `TriangulateTest` (13),
`NavMeshTest` (18), `PathTest` (10), `SimulationTest` (21) = 78, all deterministic (fixed
inputs, fixed iteration order, no time/randomness/containers keyed by floats).

## 4. Files

* Added: `src/{predicates.hpp, geometry.cpp, triangulation.{hpp,cpp}, mesh_topology.{hpp,cpp},
  nav_mesh_impl.hpp, nav_mesh.cpp, corridor.{hpp,cpp}, pathfinding.cpp, agent.hpp,
  steering.{hpp,cpp}, simulation.cpp, error.hpp}`, `tests/{mesh_fixtures.hpp, geometry_test.cpp,
  triangulation_test.cpp, nav_mesh_test.cpp, path_test.cpp, simulation_test.cpp}`,
  `tests/consumer/{CMakeLists.txt, main.cpp, README.md}`, `scripts/{check.sh, fmt.sh}`,
  `.clang-format`, `docs/architecture/{architecture.md, implementation-plan.md}`,
  `docs/IMPLEMENTATION_REPORT.md`.
* Modified: `CMakeLists.txt` (library sources, strict warnings, alias `vwmini::vwmini`, C++23,
  `include(CTest)` + `BUILD_TESTING` with `VWMINI_BUILD_TESTS` as an alias),
  `tests/CMakeLists.txt`, `tests/mesh_fixtures.hpp` (`offset` helper),
  `tests/consumer/main.cpp` (extreme-coordinate checks), `README.md`,
  `include/vwmini/nav_mesh.hpp` (private `friend` declaration only).
* Deleted: placeholder `src/vwmini.cpp`.
* Requirements, public API doc and public declarations: unchanged.

## 5. Contract points where more than one implementation is legal — my choices

* **Route shortening**: verified greedy "jump to furthest visible waypoint" over a
  portal-seeded polyline; taut but not proven optimal (SIM-003/SIM-004 allow this).
* **Winner among equal-cost routes**: not prescribed — Dijkstra ties break on cell index.
* **Segment containment**: the critical-parameter method - every edge crossing of the query
  segment is collected, and the midpoint of each parameter interval is tested for containment,
  which decides coverage of the whole segment (`MeshTopology::segment_is_contained`).
* **Avoidance**: collision-cone deflection with a deterministic side choice (clockwise for exactly
  head-on pairs), positional separation for initially overlapping discs (SIM-010 edge case), fixed
  axis for exactly concentric discs; no braking, only a max-speed clamp.
* **Substepping**: `min(seconds / kPreferredSubstep + 1, kMaxSubsteps)` — a pure function of the
  argument; large durations cannot overshoot a waypoint or goal.
* **Norm at the limit**: `length` saturates at `FLT_MAX` rather than returning `inf` for a finite
  vector whose true norm is unrepresentable. Saturating is the only choice that keeps
  "finite input -> finite output" true; returning `inf` would break that promise, and clamping in
  the other direction would silently change ordinary magnitudes. The alternative — refusing such
  vectors — is not allowed, because the contract accepts any finite coordinates.
* **`agent_count` exactness**: ids are monotonic and never reused, removal erases from the table,
  so `agent_count` is the table size, a removed id keeps returning `NotFound`, and the table index
  stays the deterministic iteration order.
* **Tolerances**: contract `epsilon` reused for the two route-state tolerances that measure the
  same scale; every other tolerance is its own named constant.

## 6. Numeric-scale safety repair

**What was wrong.** Internal geometry computed products, subtractions and accumulations in
`float`. For accepted finite coordinates that is not safe: with a triangle
`(0,0) (3e38,0) (0,3e38)` the edge cross product that decides inside/outside is `inf` in float
(`3e76` in double), so the sign tests degenerate; `2.5e38 + 1.5e38` is `inf` in float (`4e38`
in double); squared lengths overflow (`(3e38)^2` -> `inf`). `length`/`normalized` returned
non-finite values or collapsed to `{0,0}`. Those are inputs the contract accepts, so a decision
built on an overflowed intermediate is a defect, not an edge case.

**Change, per requirement.**

1. *Single documented policy* — `src/predicates.hpp` now states the policy at the top and is the
   only module that performs geometric arithmetic; `src/vec2d.hpp` holds `Vec2d` (double),
   implicitly widened from `Vec2`. `epsilon` is used for point/edge distance, endpoint matching and
   waypoint omission (`kEpsilon`, `kEpsilonLength`, `kEpsilonSquared`), plus the two route-state
   rules of §5 that measure the same metre scale; no orientation or area tolerance was
   introduced.
2. `signed_double_area`, `ring_double_area` and squared-length helpers return `double`; ear and
   validation tests compare against `kMinNonDegenerateDoubleArea` = `epsilon * epsilon` (no new
   tolerance).
3. `length` evaluates `hypot` in double and saturates at `FLT_MAX` for a finite vector whose true
   norm is unrepresentable; non-finite input propagates.
4. `normalized` divides by the largest-magnitude component first, so an extreme or denormal vector
   still gets a unit direction; only zero/non-finite input yields `{0,0}`.
5. Containment (`MeshTopology::cell_contains`, `segment_is_contained` including its
   critical-parameter samples), topology predicates (`endpoints_coincide`,
   `point_inside_edge_span`, `edges_match`), corridor Dijkstra costs and cell centroids, waypoint
   dropping (`pathfinding.cpp`, `agent.hpp`) and the motion/avoidance step (`steering.cpp`,
   `simulation.cpp`) all decide on doubles. `Triangle::centroid()` returns `Vec2d` because the sum
   of three float coordinates overflows. A substep is applied only when the new position is a
   finite `Vec2` (SIM-012 shortening), otherwise the agent does not move that substep.
6. `CMakeLists.txt` uses `include(CTest)`; the suite follows `BUILD_TESTING` (verified OFF:
   library-only build), with `VWMINI_BUILD_TESTS` kept as an alias.
7. `tests/numeric_test.cpp` adds nine GTest regressions that are active in Release: extreme
   containment, extreme path queries, extreme `length`/`normalized`, routing and motion on meshes
   translated far from the origin, and - from the review round - a straight route across a strip
   wider than the float range, steering towards a goal further away than the float range, and the
   exact `max_speed` cap. The consumer gained equivalent checks.
8. README, `docs/architecture/architecture.md` §6 and this report describe the policy as
   implemented; the numbers above are the recorded outcomes of the commands in §3.

**Evidence that the policy changes behaviour** (compiled snippet, same expressions both ways):

```text
cross(float)  = inf        cross(double) = 3e+76
2.5e38+1.5e38 float = inf   double = 4e+38
(3e38)^2 float = inf       double = 9e+76
```

**Not claimed:** the numeric regressions cover the extreme-coordinate behaviours named above;
they are not a proof of absence for other numeric defects, and the taut-route algorithm remains
"taut by verification", not proven optimal.

## Review round: numeric scale, second pass

A read-only review of the first repair found further places where a decision still rested on a
`float` intermediate, plus policy inconsistencies. All were fixed. Three of them have named
regressions in `tests/numeric_test.cpp` (straight route across a strip wider than the float range,
steering towards a goal further away than the float range, exact `max_speed` cap); the rest —
collinear overlap reporting, the substep sweep check, the shoelace pivot, the substep count, the
relative steering fractions, the route-verification fallback — are covered by the multi-configuration
gate below and by the existing containment/avoidance suites rather than by a dedicated case.

| Finding | Where | Fix |
|---|---|---|
| Segment delta computed in `float`: a span over `FLT_MAX` overflowed, so a fully contained segment was reported not walkable and a straight route was reported as a four-point detour | `mesh_topology.cpp` (`segment_is_contained`, `endpoints_coincide`) | Widen both endpoints before subtracting; regression `RouteAcrossAStripWiderThanTheFloatRangeIsStraight` |
| Steering offset computed in `float`: a goal further away than `FLT_MAX` overflowed to `inf`, `normalized` returned `{0,0}` and the agent stalled at zero velocity while `Moving` | `steering.cpp` (`seek_velocity`) | Offset subtracted in double, direction from the new double-domain `unit()` helper; regression `AgentSteersAcrossAnOffsetWiderThanTheFloatRange` |
| Waypoint-drop decision fed by a `float` difference | `pathfinding.cpp` | Operands widened individually |
| Stored velocity could exceed `max_speed` by a float ULP after narrowing the clamped double | `steering.cpp` (`to_velocity`) | Re-check the narrowed `Vec2` against the cap and pull it down; regression `StoredVelocityDoesNotExceedTheConfiguredMaximumSpeed` |
| Collinear overlap reported only the start of the shared span, so a coverage sample could straddle covered and uncovered ground | `predicates.hpp` | old
`segment_intersection_parameter` (removed) replaced by `append_crossing_parameters`, which reports both ends of the overlap |
| Substep motion tested containment at the candidate endpoint only; all candidates lie on one ray, so a thin obstacle could be crossed inside one substep | `simulation.cpp` (`contained_displacement`) | Accept a candidate only when both ends and the midpoint of the swept segment are inside the mesh, via `NavMesh`'s public point query |
| Motion below one float ULP of the coordinate was silently lost, freezing agents far from the origin | `simulation.cpp`, `agent.hpp` | New `AgentRuntime::residual` carries the unrepresentable remainder to the next substep |
| Substep count computed as `seconds / kPreferredSubstep` in `float`: overflowed for durations near `FLT_MAX` | `simulation.cpp` | Counted in double, then clamped to `kMaxSubsteps` |
| Absolute steering thresholds (1e-6) unrelated to the pair's own scale | `steering.cpp` | `kStillSpeedFraction` / `kConcentricFraction`, fractions of the pair's speed and radii |
| `kEpsilonSquared` aliased the degenerate-area threshold; an orphaned doc comment described a route tolerance that no longer exists as its own constant | `predicates.hpp`, `simulation.cpp`, architecture doc | Independent constant; orphan comment deleted; tolerance list in `architecture.md` corrected |
| Ring shoelace accumulated absolute coordinates (cancellation, not translation invariant) | `triangulation.cpp` | Shoelace measured from the ring's first vertex |
| Shortened route handed back unverified when re-dropping failed verification | `pathfinding.cpp` | Pulled route verified too; falls back to the seeded corridor polyline, which is contained by construction |

`length` propagates non-finite input while `normalized` maps it to `{0, 0}`. That asymmetry is
deliberate and documented: `normalized` is used to obtain a steering direction, and a zero direction
is the documented "no direction" answer. The internal `unit()` helper follows the same rule.

### Verification after the second pass

| Check | Result |
|---|---|
| `scripts/check.sh` (configure, build, tests, format) | ALL CHECKS PASSED, 78/78 tests |
| Release (`NDEBUG`) build and tests | warning-free, 78/78 |
| Clang 22.1.8 Debug build (`-Wall -Wextra -Wpedantic` + strict set) | warning-free, 78/78 |
| ASan + UBSan build | 78/78, no sanitizer reports |
| `-DBUILD_TESTING=OFF` library-only configure | library only, warning-free |
| Consumer project (`tests/consumer`, `BUILD_TESTING=OFF`) | builds warning-free, exits 0 |
| `clang-format --dry-run -Werror` over `include/`, `src/`, `tests/` | clean |
