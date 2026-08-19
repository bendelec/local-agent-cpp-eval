# VWmini — Completion Report

## Summary

Implemented **VWmini** from scratch: a compact native C++23 library providing an immutable 2D navmesh, deterministic point-to-point paths, and local disc-agent avoidance. The fixed public API in `include/vwmini/` and the `vwmini::vwmini` CMake target were preserved unchanged; only private implementation details were added to headers.

## Design / Modules

Three single-responsibility modules with dependency flow inward (`geometry → navmesh → simulation`), each owning its private `Impl` (referenced by pointer/shared_ptr from the public headers to preserve the fixed API).

| Module | Files | Responsibility |
|---|---|---|
| `geometry` | `src/geometry.cpp` | `Vec2`/`Polygon` ops, `triangulate_simple_polygon` via deterministic ear-clipping. |
| `navmesh` | `src/internal.hpp`, `src/nav_mesh.cpp` | Shared helpers; `NavMesh::create` validation, `contains`, `cell_count`, `find_path` (cell A* corridor + visibility-graph Dijkstra corridor shortening). |
| `simulation` | `src/simulation.cpp` | Move-only `Simulation`: agent lifecycle, substep motion, snapshot-based reciprocal swept-segment avoidance. |

### Key architectural seams (documented in `docs/architecture/architecture.md`)
- **Validation seam**: every public input is validated; `Result<T>`/`Error` with the fixed `ErrorCode` values; never crashes or silently repairs invalid geometry.
- **Boundary-policy seam**: `contains` uses strict interior + closed-edge distance ≤ epsilon.
- **Adjacency seam**: adjacency only via complete non-zero shared edges with endpoints ≤ epsilon apart; vertex-only touch creates no adjacency.
- **Determinism seam**: all algorithms are deterministic; identical input → identical output (triangulation, `cell_count`, paths, simulation runs).

### Avoidance design (final)
Reciprocal **swept-segment** prediction: each agent evaluates candidate velocity offsets against the *other agent's swept segment* (its snapshot position → predicted position over the lookahead horizon at snapshot velocity, `kHorizon = 0.5f`), so agents swerve early. `kOffsets` are ordered to prefer large lateral swerves (90°/-90°/45°/-45°/…), so head-on/crossing agents diverge symmetrically without deadlock. The required per-pair clearance is relaxed progressively from the strict margin down to touching distance, so crossing/overtaking agents can slip past instead of stalling. Stop (`{0,0}`) is the final fallback (SIM-012 permits stopping when boxed). A post-decision `resolveCommitted` pass verifies the *committed* substep swept segments directly and re-decides the higher-index agent of any overlapping pair, so the committed segments stay separated whenever collision-free motion exists — independent of the snapshot/horizon discrepancy.

## Files Changed

- `src/vwmini.cpp`, `src/geometry.cpp`, `src/internal.hpp`, `src/nav_mesh.cpp`, `src/simulation.cpp` — implementations.
- `CMakeLists.txt`, `tests/CMakeLists.txt` — library sources and three test executables.
- `tests/geometry_test.cpp`, `tests/nav_mesh_test.cpp`, `tests/simulation_test.cpp` — deterministic tests.
- `docs/architecture/architecture.md` — architecture doc (required, created from scratch, kept current).
- `.clang-format` — formatting config (LLVM-based, 120 column limit).

## External Verification Pass — Fixes Applied

An external pass reported six issues; each was reproduced, fixed, and a regression test added. The public API, error codes, and documented boundary semantics were preserved.

| Issue | Root cause | Fix | Regression test |
|---|---|---|---|
| 1. Direct route with boundary-tolerance endpoints returned 5 points | `directContained` used only proper boundary-edge crossing, rejecting a segment that crosses an edge within the tolerance zone | Dense-sampling containment check (MSH-006/SIM-002): accept iff every sample point is inside a triangle or within `epsilon` of an edge | `test_find_path_direct_boundary_tolerance` (exactly 2 endpoints) |
| 2. L-shape route had a segment outside the mesh | Corridor visibility used proper-crossing only, accepting a diagonal across the non-walkable notch (tangential contacts) | Corridor visibility edges must pass dense mesh-containment sampling | `test_find_path_segments_contained` (every segment densely sampled) |
| 3. Crossing left both agents `Moving` | Both agents stopped at the strict margin and deadlocked (no candidate) | Progressive margin relaxation down to touching before the stop fallback | `test_avoidance_crossing_large_radius` (radius 0.5, both reach goals) |
| 4. Movement-commit mismatch + endpoint-only mesh check | `applyMove` snapped to a waypoint whenever `|disp| ≥ remLen` regardless of direction; containment checked only at the endpoint | Snap only when the displacement projects past the waypoint along the route direction; check the whole swept displacement is mesh-contained | swept-membership + bounded-motion checks in the large-radius crossing test |
| 5. Identical overlapping CCW triangles accepted | Interior-overlap test missed collinear edge overlap with interiors on the same side | Added collinear-overlap + same-side detection; valid shared-edge adjacency stays accepted | `test_create_identical_triangles` (`InvalidMesh`) |
| 6. Self-touching / overlapping collinear outlines accepted | Self-intersection test missed endpoint-on-edge and collinear-overlap contacts | Polygon validation rejects point-on-segment and collinear-overlap for non-adjacent edges | `test_triangulate_self_touch`, `test_triangulate_collinear_overlap` (`InvalidMesh`) |

## Second External Verification Pass — Fixes Applied

A second external pass reported six further issues; each was diagnosed, fixed, and a regression test added. The public API, error codes, and documented boundary semantics were preserved.

| Issue | Root cause | Fix | Regression test |
|---|---|---|---|
| 1. L-shaped route returned goal, then goal again as consecutive final point | `corridorPath` pushed `goal` explicitly but the reversed parent chain still ended with the goal node (index 1) | Drop that duplicate goal node from the chain after `std::reverse` before pushing `goal` | `test_find_path_l_bend` (goal appears exactly once as final point, no consecutive duplicate) |
| 2. Sampling-based segment containment could disagree with the continuous contract | Dense sampling with a fixed cap can miss a narrow non-walkable gap on a long segment | Exact interval-coverage continuous containment (convex `distToTri` + ternary minimizer + two monotone bisections; merge covered intervals, check `[0,1]` coverage); sampling retained only for short swept segments | `test_find_path_narrow_gap` (U-shape 0.02-wide mouth → corridor, not direct), `test_find_path_segments_contained` (notch diagonal returns ≥ 3 points) |
| 3. Two-agent open-space crossing brought disc centres closer than combined radii despite candidate checks succeeding | Committed-vs-snapshot discrepancy: a candidate cleared the other's snapshot-horizon swept segment, but the other's actually committed substep motion can differ when both swerve | Post-decision `resolveCommitted` re-decides the higher-index agent of any overlapping committed pair via `findCommittedCandidate` (clears every other's committed swept segment, progress + mesh-containment), stop fallback, rounds bounded | `test_avoidance_crossing_swept` (per-substep committed swept-segment separation) |
| 4. `find_path` with identical finite endpoints outside the mesh succeeded | `contains(start)`/`contains(goal)` checks were ordered after the equal-endpoint special case | Reordered: finite → `contains(start)` → `contains(goal)` → equal-endpoint special case | `test_find_path_direct` (identical finite endpoints outside → `OutsideMesh`; in-mesh → `[start]`) |
| 5. Mesh overlap validation rejected a valid pair of thin opposite-side triangles sharing a complete edge | Tolerance-based side classification flattened near-line third vertices onto the shared edge, wrongly judging interiors same-side | Exact signed-area side classification (`triArea2 > 0 ? 1 : -1`), removing the tolerance helper | `test_create_thin_opposite_triangles` (accepted, 2 cells); `test_create_identical_triangles` still rejects |
| 6. `length` returned infinity for finite large coordinates | Naive `sqrt(x*x + y*y)` overflowed for large finite coords | Scaled norm (`m=max(|x|,|y|)`, `m*sqrt((x/m)²+(y/m)²)`); scaled-cross `triArea2`; normalized-direction distance/collinearity; guarded sampling cast | `test_length_large_finite` (finite for `1e20`/`1e-20`, `length(3,4)==5`) |

## Build & Test Commands + Outcomes

```
cmake -S . -B build && cmake --build build --parallel
```
Clean build, **no compiler warnings** (`-Wall -Wextra -Wpedantic`).

```
ctest --test-dir build
```
```
1/3 geometry_test .... Passed
2/3 navmesh_test ..... Passed
3/3 sim_test ......... Passed
100% tests passed out of 3
```
Individual executables: `./build/tests/vwmini_geometry_test`, `./build/tests/vwmini_navmesh_test`, `./build/tests/vwmini_sim_test` — all pass.

**Formatting**: all `src/*.cpp`, `src/*.hpp`, `include/vwmini/*.hpp`, `tests/*.cpp` pass `clang-format --dry-run --Werror` (clean).

## Deliberately-Chosen Behaviors (contract permitted choices)

1. **Ear-clipping triangulation** over alternatives — simple, deterministic, matches the single-simple-polygon authoring helper; strict ear test avoids T-junctions so output feeds `NavMesh::create` cleanly.
2. **Visibility-graph Dijkstra corridor shortening instead of the full funnel** — builds a visibility graph over corridor vertices + start/goal and runs Dijkstra to shorten the A* corridor; satisfies SIM-003 and is provably contained, with lower subtle-bug risk than the textbook funnel.
3. **Exact interval-coverage continuous containment** for the direct-segment/corridor-visibility check (SIM-001/002) — because `distToTri` is convex along a segment, ternary + two monotone bisections find the covered interval exactly to float precision; merging per-triangle intervals and checking `[0,1]` coverage decides containment with no sampling cap. Admits boundary-tolerance segments while rejecting genuine mesh exits (e.g. a narrow non-walkable gap). Dense sampling is retained only for the short swept-membership segments bounded by `max_speed*dt`.
4. **Same-cell endpoints → `[start, goal]` directly** — avoids inconsistency between the containment check and the corridor path in the tolerance-ambiguous single-cell case.
5. **Substep loop with snapshot avoidance** (SIM-008/010/012) — all agents snapshot, decide, then apply displacements simultaneously.
6. **Reciprocal swept-segment avoidance with progressive margin relaxation + committed-segment verification** — swept-segment prediction with a lookahead horizon makes head-on/crossing agents diverge deterministically; the clearance is relaxed down to touching so feasible crossings pass, and the stop-fallback guarantees no overlap. A post-decision `resolveCommitted` pass re-decides the higher-index agent of any overlapping committed pair (via `findCommittedCandidate`), guaranteeing committed swept-segment separation whenever collision-free motion exists (SIM-011).
7. **Arrival-radius rule** (SIM-006): only `-1.0f` is the negative sentinel; effective arrival = radius when `-1.0f`, else the given radius; within effective arrival → `Reached`.
8. **`routeIndex` starts at 1** in `assignGoal` (skips `route[0]`, the already-occupied start point) — fixes multi-substep oscillation toward the start.
9. **`nextId` wraps to 1 after `UINT32_MAX`** to avoid overflow UB.
10. **Numerical policy**: epsilon `1e-4f` applies only to point/edge distance and shared-edge endpoint matching; triangle non-degenerate iff absolute signed double area > `epsilon²`; ambiguous inputs separated ≤ epsilon may accept or reject but stay safe and deterministic. `length`/`triArea2`/distance helpers use scaled/normalized forms so finite large/small coordinates stay finite and signs stay exact.
