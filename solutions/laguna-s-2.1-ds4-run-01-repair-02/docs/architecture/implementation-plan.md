# Implementation Plan

Ordered work packages (WPs) broken into verifiable slices. Each slice lists goal,
dependencies, expected files/interfaces touched, verification, and completion status.
Status values: `planned → in-progress → done`. Revise here when evidence changes design.

Legend for verification commands:
- build: `cmake -S . -B build && cmake --build build --parallel`
- format check: `clang-format --dry-run --Werror -i <files>` (style baseline `.clang-format`)
- tidy: `clang-tidy src/*.cpp include/vwmini/*.hpp`
- test: `ctest --test-dir build --output-on-failure`

---

## WP1 — Geometry primitives  (foundation)

**Goal:** Provide correct, noexcept vector math used everywhere else.

| Slice | Goal | Deps | Files | Verification | Status |
|---|---|---|---|---|
| 1.1 | Implement `length`, `normalized`; add numeric helpers | header contract | `src/geometry.cpp`, `include/vwmini/geometry.hpp`(add private inline helpers if needed) | unit checks: length({3,4})==5; normalized({0,0})=={0,0}; cross/dot signs | done |
| 1.2 | Add `.clang-format` + compile flags `-Wall -Wextra -Wpedantic` | CMake | `CMakeLists.txt`, `.clang-format` | warning-clean compile of an empty TU linking vwmini | done |

## WP2 — Triangulation  (mesh authoring)

**Goal:** `triangulate_simple_polygon` validate+triangulate one simple CCW hole-free outline.

| Slice | Goal | Deps | Files | Verification | Status |
|---|---|---|---|---|---|
| 2.1 | Finiteness check (InvalidArgument); vertex count & cyclic-duplicate & winding & area validation (InvalidMesh) | WP1 | `src/triangulator.cpp` | tests: non-finite→InvalidArg; n<3/dup/cyclic-dup/CW/zero-area→InvalidMesh | done |
| 2.2 | Self-intersection rejection | WP1 | `src/triangulator.cpp` | bowtie → InvalidMesh; valid square/pentagon OK | done |
| 2.3 | Ear-clipping core emitting CCW non-degenerate triangles; collinear verts handled | WP1 | `src/triangulator.cpp` | triangle/square/pentagon/collinear-chain produce non-overlapping CCW tris; area sum ≈ input; deterministic repeat | done |

## WP3 — NavMesh construction & queries

**Goal:** immutable mesh, adjacency, containment.

| Slice | Goal | Deps | Files | Verification | Status |
|---|---|---|---|---|---|
| 3.1 | `Impl` layout + `create` finiteness(non-fin)/empty/non-triangle/CW/degenerate checks | WP1 | `src/nav_mesh.cpp`,`nav_mesh.hpp`(friend/find_path decl unchanged) | single bad triangle rejected per code; good set accepted | done |
| 3.2 | Pairwise overlap/T-junction/non-manifold-edge detection | WP1 | `src/nav_mesh.cpp` | overlapping interiors / T-junction / >2 shared edges → InvalidMesh; disjoint components allowed | done |
| 3.3 | Adjacency table derivation (|MSH-005|) | WP1 | `src/nav_mesh.cpp` | shared complete edge ↔ adjacent; vertex-only touch not | done |
| 3.4 | `contains` per MSH-006 (strict-inside OR dist-to-edge-seg ≤ eps), const allocation-free | WP3.1 | `src/nav_mesh.cpp` | interior true; outside far false; near-boundary within eps true; non-finite false | done |
| 3.5 | `cell_count` | WP3.1 | `src/nav_mesh.cpp` | equals accepted triangle count | done |

## WP4 — Pathfinding

**Goal:** deterministic shortest contained route via visibility graph + Dijkstra.

| Slice | Goal | Deps | Files | Verification | Status |
|---|---|---|---|---|---|
| 4.1 | Segment-cover helper (segment inside union of closed triangles, tolerant merge) | WP3 | `src/navmesh_detail.hpp`,`nav_mesh.cpp` | full cover vs gap detection on hand examples; analytical half-plane interval clipping replaces capped sampler | done |
| 4.2 | Endpoint validation + OutsideMesh/NoPath/direct-path shortcut `[start]`/`[start,goal]` | WP3,WP4.1 | `nav_mesh.cpp` (`find_path`) | SIM-001 cases exact: non-finite→InvalidArg, out-of-mesh→OutsideMesh, equal endpoints→`[start]`, direct LOS→`[start,goal]` | done |
| 4.3 | Visibility-graph nodes {start,goal}+verts, edges=contained segments; Dijkstra deterministic tie-break; adjacency BFS connectivity gate | WP4.1,WP3 | `nav_mesh.cpp` (`find_path`) | multi-cell bent path contained & deterministic across repeats; disconnected components→NoPath; no routing across vertex-only touches | done |

## WP5 — Simulation lifecycle & state

**Goal:** agent storage, add/remove/set_goal/clear_goal/agent/agent_count with all transitions.

| Slice | Goal | Deps | Files | Verification | Status |
|---|---|---|---|---|---|
| 5.1 | `Simulation::Impl` skeleton: holds NavMesh copy + agents; move-only dtor/move defined in cpp | WP3 | `src/simulation.cpp` | compiles; agent_count starts 0 | done |
| 5.2 | Config validation + add_agent transitions (Moving/Reached/NoPath/OutsideMesh/InvalidArg) | WP4,WP5.1 | `src/simulation.cpp` | each branch returns right error/status | done |
| 5.3 | remove_agent/set_goal/clear_goal/agent/agent_count semantics (NotFound, Idle) | WP5.2 | `src/simulation.cpp` | removed id→NotFound; clear→Idle zero vel snapshot | done |

## WP6 — Stepping & local avoidance

**Goal:** motion integration respecting speed/containment/no-overshoot plus disc avoidance.

| Slice | Goal | Deps | Files | Verification | Status |
|---|---|---|---|---|---|
| 6.1 | step() validation + substep subdivision + goal/waypoint clamping + mesh projection + arrival | WP5 | `src/simulation.cpp` | invalid duration no-mutation; Reached at terminal; never exceeds max_speed*dt; stays in mesh | done |
| 6.2 | Snapshot-then-update steering with symmetric pairwise repulsion + residual overlap correction | WP6.1 | `src/simulation.cpp` | two crossing open-space agents stay separated ≤1e-3 and progress; NoPath/idle stationary | done |

## WP7 — Build, tests, quality gate

| Slice | Goal | Deps | Files | Verification | Status |
|---|---|---|---|---|---|
| 7.1 | CMake self-contained framework + `BUILD_TESTING` guard; ctest registered only when tests enabled | build chain | `CMakeLists.txt`, `tests/*` | `cmake -S . -B build && cmake --build build` builds lib-only; with `-DBUILD_TESTING=ON` ctest green | done |
| 7.2 | Full test sweep + clang-format -i clean + warning-clean rebuild on GCC and Clang, ASan/UBSan gate | all | whole tree | final commands documented below | done |

### Final verification (WP7)
- `cmake -S . -B build && cmake --build build --parallel && ctest --test-dir build --output-on-failure` → 5 groups / 0 failures, `-Wall -Wextra -Wpedantic` clean under GCC 16.2.1.
- `.clang-format` accepted by system clang-format 22.1.8 (`Standard: Latest`, `IncludeBlocks: Preserve`); all active sources `FMT-OK`.
- Clang 22.1.8 configure+build with `-fsanitize=address,undefined -fno-omit-frame-pointer` passes the full suite (leak detection disabled for process-exit paths).

---

## Notes / revision log
- Plan is bottom-up so each slice builds on a compiling base. Visibility-graph + analytical segment-cover replaces the capped sampler: direct visibility yields exactly `[start,goal]` naturally, and deterministic Dijkstra ascending-index tie-break gives stable multi-cell routes gated by adjacency BFS (no routing across vertex-only touches). `find_path` lives in `nav_mesh.cpp` as a free function friended into `NavMesh::Impl`; no public seam/Triangle/`span`/friendship beyond that single trusted query. Simulation uses snapshot-then-update steering with symmetric positional separation and mesh backtracking projection.
