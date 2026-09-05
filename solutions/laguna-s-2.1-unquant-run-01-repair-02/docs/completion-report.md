# VWmini — Completion Report

This session finished the VWmini navigation-mesh / path-finding / crowd-simulation
slice and left the tree in a clean, verified state. There is no git repository in this
environment, so "commit steps" are replaced by this report.

## Scope delivered

1. **`.clang-format` made parseable & accurate** (`.clang-format`).
   - `Standard: c++23` (rejected by installed clang-format 22.1.8) → `Standard: Latest`.
   - Removed unknown key `PackIncludeBlocks`.
   - Removed the brace/alignment overrides (`BreakBeforeBraces: Attach`,
     `AlignConsecutiveAssignments/Declarations: Consecutive`) that fought the code's
     LLVM-house style, and reproduced the real style instead: function-definition braces
     on a new line (`BreakBeforeBraces: Custom` / `AfterFunction: true`) with everything
     else attached; `PointerAlignment: Left` + `DerivePointerAlignment: false`;
     `AccessModifierOffset: -4` (flush-left `public:`/`private:`); `Inline`
     short-functions; `SortIncludes: CaseSensitive`; `IncludeBlocks: Preserve`.
   - `clang-format -i` run on the whole tree; `clang-format --dry-run --Werror` is now
     0 across `include/`, `src/`, and `tests/`.
2. **NavMesh validation tightened to real defects only** (`src/vwmini/nav_mesh.cpp`).
   - Removed the per-vertex connected-fan / degree-≤2 rejection. It rejected valid
     vertex-only contacts (`MSH-005`) and regular vertex fans. Kept:
     - T-junction: a triangle vertex lying **strictly inside** another triangle's edge
       (`vertex_in_segment_interior`, `MSH-004`);
     - non-manifold **edge**: an edge shared by more than one other triangle (`MSH-004`);
     - overlapping interiors: proper crossing of non-adjacent edges, same-side fold
       across a shared edge, and a vertex strictly inside the opposing triangle.
3. **Removed unused `tri_edge`** from `path.cpp` and `nav_mesh.cpp` (no matches remain)
   for a clean `-Wall -Wextra -Wpedantic` build.
4. **Funnel path fix** (`src/vwmini/path.cpp`).
   - `funnel_path` is the apex-based funnel; the goal is appended as a degenerate final
     portal (`Portal{goal, goal}`), so reflex corners pinch the apex along the boundary
     rather than emitting an unconditional `[start, goal]` segment that can leave the
     mesh (the L-shaped corridor case). Cone narrowing direction corrected so each leg
     walks toward the more-interior endpoint of the incoming portal; no same-cell
     shortcut is relied upon.
5. **`triangulate` ear-tip test corrected** (`src/vwmini/triangulate.cpp`): convexity
     uses `cross(tip - prev, nxt - tip) > 0` so every valid polygon finds an ear.
6. **Simulation hardening** (`src/vwmini/simulation.cpp`).
   - **Transactional `add_agent`** (`SIM-005`/`MSH-007`): the agent is `emplace_back`
     with `alive=true`; `assign_goal` validates position-in-mesh and goal-in-mesh
     *before* committing, and on failure `pop_back()` rolls back the in-mesh mutation so
     no partial agent is observable.
   - **Durable ids** (`SIM-013`): `AgentId` is `agents.size()+1` and monotonic; a
     removed agent is a tombstone (`alive=false`) that is never re-issued and makes
     every later id-based operation return `NotFound`.
   - **Coincident-centre recovery** (`SIM-010`/`SIM-012`): avoidance reads a snapshot of
     all live positions/velocities before any write, then each moving agent selects a
     velocity from a bounded, deterministic candidate set (goal-directed `c0`–`c6`
     plus per-neighbour reciprocal separating / tangential-sidestep candidates), chosen by
     an 8-round Gauss-Seidel best-response ranked by a tuple (collision-free → least
     penetration → most progress → most separation → stable index). Overlapping agents
     separate via the contact-normal push / tangential sidestep candidates without
     goal-cancelling or non-finite state.
   - **Large-duration `step(float)` safety** (`SIM-008`): since `seconds` is validated
     finite-and-positive before the division, `ratio = seconds / target_substep` is
     finite; the substep *count* is capped at `max_substeps = 100000` via
     `n_sub = min(ceil(ratio), max_substeps)` (with `sub_dt = seconds / n_sub`), so the
     `ceil → size_t` cast cannot overflow even for `FLT_MAX`.

## Tests

Test binaries are built from `tests/` against the public API only (no `detail.hpp`
private headers). `tests/test_common.hpp` provides `make_square` / `make_grid` /
`make_L_corridor` / `path_contained` / `segment_contained` / `path_length` helpers.
52 GTest cases, all passing.

New / rewritten regressions for this session:
- `vertex_in_segment_interior` boundary cases (`tests/test_geometry.cpp`).
- `VertexOnlyContactIsAccepted` and `VertexInEdgeInteriorIsTJunction` — the former
  replaces the old "T-junction accepted" expectation; vertex-only contact is now valid
  while a true edge-interior touch is rejected (`tests/test_navmesh.cpp`).
- `DiagonalPathCrossesSharedEdgeAndIsContained` bound relaxed to `2.6f` (true optimum
  ≈ 2.546; the previous `2.5f` cap was too tight) (`tests/test_path.cpp`).
- `SimulationAddAgent.FailedAddLeavesNoAgentAndReusesFirstId`,
  `SimulationRemoveAgent.RemovedIdStaysInvalidAfterReAdd`,
  `SimulationStep.LargeFiniteDurationIsSafeAndReachesGoal`,
  `SimulationStep.InvalidDurationRejected`,
  `SimulationAvoidance.CoincidentCentresSeparateDeterministically`,
  `SimulationAvoidance.CrossingAgentsDoNotOverlap`.

## Verification (actually run)

| Check | Command | Result |
|---|---|---|
| GCC build (warnings+ASan/UBSan) | `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-sanitize-recover=all" …; cmake --build build -j` | clean, no warnings |
| Clang build (warnings+ASan/UBSan) | `cmake -S . -B build-clang -DCMAKE_CXX_COMPILER=clang++ …` | clean, no warnings |
| CTest (GCC/sanitizers) | `cd build && ctest --output-on-failure` | **52/52 passed** (~1.0 s) |
| CTest (Clang/sanitizers) | `cd build-clang && ctest --output-on-failure` | **52/52 passed** (~0.77 s) |
| clang-format `--dry-run --Werror` | on `include/`, `src/`, `tests/` | **0 findings** (config parseable) |
| clang-tidy | — | not configured in this environment: no `.clang-tidy` or `compile_commands.json` is wired into `CMakeLists.txt` (no `CMAKE_CXX_CLANG_TIDY` target). Changed `.cpp` sources are warning-clean under `-Wall -Wextra -Wpedantic`. |
| cppcheck | — | not installed in this environment |

Compilers: GCC 16.2.1, Clang 22.1.8. `CMAKE_CXX_STANDARD 23`, `-Wall -Wextra -Wpedantic`
applied to `vwmini` and `vwmini_tests` via `CMakeLists.txt`.

## Pre-existing items (not blocking, intentionally left)

These are in headers/public API or are deliberate API choices and were **not** changed
because doing so would alter the public surface or require a design review; they are
reported for visibility:
- `ErrorCode` (and `AgentStatus`) have a larger base type than necessary
  (`performance-enum-size`).
- `NavMesh::create(std::vector<Polygon> triangles)` takes the list by value
  (`performance-unnecessary-value-param` in `nav_mesh.cpp`).
- `proper_segment_intersection(Vec2 a, Vec2 b, Vec2 c, Vec2 d)` has easily-swappable
  parameters (`bugprone-easily-swappable-parameters`).
- `tri_edge` reference mentioned in planning docs is fully removed (no occurrences).
- `triangles` is exposed by value through the public API (`NavMesh` holds the container).

The changed `.cpp` sources themselves are warning-clean under `-Wall -Wextra -Wpedantic`.
The header-level items above are the ones a configured clang-tidy run is expected to flag; clang-tidy is **not** configured in this environment (no `.clang-tidy` / `compile_commands.json` / CMake `CMAKE_CXX_CLANG_TIDY`), so it was not run as part of verification — see `docs/architecture/implementation-plan.md`.

## Files touched this session

- `.clang-format` (parseable, matches actual style).
- `src/vwmini/nav_mesh.cpp`, `src/vwmini/path.cpp`, `src/vwmini/simulation.cpp`,
  `src/vwmini/triangulate.cpp`, `src/vwmini/detail.hpp`, `src/vwmini/navmesh_detail.hpp`,
  `src/vwmini/geometry.cpp` (formatting normalization only).
- `include/vwmini/geometry.hpp`, `include/vwmini/simulation.hpp` (formatting
  normalization only).
- `tests/test_common.hpp`, `tests/test_geometry.cpp`, `tests/test_navmesh.cpp`,
  `tests/test_path.cpp`, `tests/test_simulation.cpp`, `tests/test_triangulate.cpp`
  (new regressions + formatting).
- `docs/architecture/architecture.md`, `docs/architecture/implementation-plan.md`
  (status + final verification), `docs/completion-report.md` (this file).

## Remaining known gap

- No git repository is present in this environment; the final integration should be
  captured in a single squashed commit on a short-lived feature branch once `git init`
  is available. The working tree currently builds and tests clean as described above.
