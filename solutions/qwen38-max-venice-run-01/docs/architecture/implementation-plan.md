# VWmini Implementation Plan

Ordered work packages (WP) decomposed into small verifiable slices. Status:
`[ ]` pending, `[x]` done, `[~]` in progress. Revisions are recorded explicitly
at the bottom; slices are not silently dropped.

## WP0 — Scaffolding

- [x] **S0.1 Git init + baseline build.** Goal: repo tracks supplied contract;
  `cmake -S . -B build && cmake --build build` succeeds with the stub. Files:
  `.gitignore`, `CMakeLists.txt`. Verify: configure+build clean.
- [x] **S0.2 Test harness.** Goal: CTest wired to system GTest with one trivial
  passing test. Files: `CMakeLists.txt`, `tests/CMakeLists.txt` (or inline),
  `tests/smoke_test.cpp`. Verify: `ctest` runs and passes. Warnings
  `-Wall -Wextra -Wpedantic` enabled for all targets.

## WP1 — Geometry core (MSH-001, MSH-002/003)

- [x] **S1.1 `internal/predicates.hpp` + `geometry.cpp`.** Goal: `length`,
  `normalized({0,0}) == {0,0}`; predicates: `is_finite(Vec2/float)`,
  `orient(a,b,c)` (double cross), `dist_point_segment`, `segments_intersect`,
  `signed_double_area`. Deps: S0. Verify: `tests/geometry_test.cpp` green.
- [x] **S1.2 `triangulation.cpp` — validation.** Goal: full MSH-002 rejection
  matrix (non-finite → InvalidArgument; <3 verts, cyclic consecutive dups incl.
  first==last, self-intersection, CW, degenerate area → InvalidMesh; collinear
  non-dup vertices allowed). Deps: S1.1. Verify: `tests/triangulation_test.cpp`
  error cases green.
- [x] **S1.3 `triangulation.cpp` — ear clipping.** Goal: deterministic CCW
  non-degenerate output, input vertices only, area preserved within
  `eps*max(1,A)` (MSH-003); collinear pre-pass; identical input → identical
  output. Deps: S1.2. Verify: square/L-shape/collinear cases + determinism.
- [x] **S1.4 simplify-check pass** on WP1 files; fix findings; rebuild+test.

## WP2 — Nav mesh (MSH-004..008)

- [x] **S2.1 `internal/mesh_data.hpp` + `nav_mesh.cpp` — validation &
  create.** Goal: define `NavMesh::Impl` (cells in caller order, dedup vertex
  table, epsilon-matched edges); reject: non-finite (InvalidArgument), empty,
  non-triangle, CW/degenerate, interior overlap, non-manifold edge, T-junction
  (InvalidMesh); accept vertex-only touch and disjoint components; build
  adjacency (MSH-005). Transactional failure. Deps: S1.1. Verify:
  `tests/nav_mesh_test.cpp` create matrix + `cell_count` + copy semantics.
- [x] **S2.2 `contains` (MSH-006).** Goal: epsilon-dilated half-plane test per
  cell; false for non-finite; const, allocation-free. Deps: S2.1. Verify:
  interior/exterior/boundary band cases (at eps inside & outside, vertex
  neighborhoods).
- [x] **S2.3 simplify-check pass** on WP2; rebuild+test.

## WP3 — Pathfinding (SIM-001..004)

- [x] **S3.1 `pathfinding.cpp` — validation & direct paths.** Goal: non-finite →
  InvalidArgument; endpoint not contained → OutsideMesh; exact-equal endpoints →
  `[start]`; contained straight segment → exactly `[start, goal]` via exact
  interval coverage: per triangle the set {t : dist(p(t), T) <= eps} is one convex
  interval (closed-interior halfplanes hull'd with three edge-capsule intervals);
  sorted merge-walk must cover [0,1]. Deps: S2.2. Verify:
  `tests/pathfinding_test.cpp` direct/error cases green.
- [x] **S3.2 Corridor search.** Goal: locate start/goal cells (first match, caller
  order), deterministic Dijkstra over adjacency (cost: portal-midpoint polyline;
  selection: smallest (cost, index); strict < keeps first predecessor; fixed edge
  order 0,1,2); disconnected → NoPath. Deps: S3.1. Verify: two/three-square
  corridors + disjoint and vertex-only-touch NoPath tests.
- [x] **S3.3 Funnel string pulling.** Goal: corridor portals (left = p[e+1],
  right = p[e] for CCW edge e of predecessor p) → shortest contained polyline via
  Mononen's simple funnel in double precision with exact vertex equality; emit
  exact float corners; assemble keeps exact start/goal and omits interior corners
  within epsilon of the previous kept point or of the goal (SIM-002/003). Deps:
  S3.2. Verify: L + mirrored-L bend exactly at reflex corner, goal-at-corner path,
  containment sampling (256/segment), determinism (repeated call + mesh copy).
- [x] **S3.4 simplify-check pass** on WP3. Outcome: shared `next_corner` /
  `cell_contains` hoisted to `mesh_data.hpp`; `std::optional` instead of sentinel
  indices; dead `restarted` flag replaced by a `pivot` lambda; shared `code_of<T>`
  test helper. 55/55 tests green, warning-clean.

## WP4 — Simulation (SIM-005..013)

- [x] **S4.1 `simulation.cpp` — lifecycle.** Goal: `Impl` (mesh, slots,
  free list, id generation), ctor/dtor/move, `add_agent` validation order
  (finite scalars → InvalidArgument; radius/max_speed > 0; arrival sentinel
  SIM-006; position/goal containment → OutsideMesh), `remove_agent`,
  `agent`, `agent_count` (SIM-013). Deps: S3.3. Verify:
  `tests/simulation_test.cpp` lifecycle + validation matrix.
  Outcome: ids are non-zero monotonic counter values, never reused; removed
  ids stay NotFound; moved-from object answers safely (NotFound/nullopt/0).
- [x] **S4.2 Goals & status transitions.** Goal: `set_goal` (NotFound /
  InvalidArgument / OutsideMesh; Moving vs immediate Reached; disconnected →
  status NoPath, velocity zero, success), `clear_goal` semantics (SIM-007),
  `add_agent` with goal applies same transitions (SIM-005). Deps: S4.1.
  Verify: transition matrix tests.
  Outcome: disconnected goals succeed with status NoPath (goal retained);
  `set_goal` to Moving deliberately leaves velocity to the next step.
- [x] **S4.3 `step` validation + single-agent movement.** Goal: negative/
  non-finite → InvalidArgument transactional; 0 → no-op; substep ≤ 1/60 s;
  route following with waypoint clamp (no overshoot, SIM-008); arrival →
  Reached + zero velocity (SIM-009); speed ≤ max_speed; positions stay
  contained (SIM-012). Deps: S4.2. Verify: straight-run, corner, big-dt,
  arrival tests.
  Outcome: uniform substeps ≤ 1/60 s capped at 4096 for huge finite inputs;
  exact arc-length route walking lands precisely on waypoints/goal.
- [x] **S4.4 Avoidance.** Goal: snapshot read before updates (SIM-010);
  deterministic candidate scoring vs neighbors; crossing & overtaking in an
  open cell: collision-free when feasible, overlap ≤ 1e-3 after any returned
  step, both reach goals (SIM-011); initially-overlapping agents stay finite
  and separate (edge case). Deps: S4.3. Verify: `tests/simulation_avoidance_test.cpp`.
  Outcome: 6 tests — crossing, overtaking, narrow-corridor deadlock
  (contained, never overlapping), initial-overlap separation, bitwise replay,
  distant-agents-no-influence (prefilter exactness). Margin
  `2·(s_i+s_j)·dt` absorbs simultaneous-replan deviation from the snapshot.
- [x] **S4.5 simplify-check pass** on WP4; rebuild+test. Outcome: the
  `contained` ranking key in `PlanScore` proved dead (the always-contained
  desired candidate anchors the comparison, so mesh-leaving deflections can
  never win) — replaced by skipping such deflections before scoring;
  `beats()` via `std::tie`; `follows_route` flag parameter removed;
  duplicated test loop extracted into `trace_until_reached`. 94/94 green,
  warning-clean.

## WP5 — Integration & quality gates

- [x] **S5.1 `tests/integration_test.cpp`.** triangulate → create → find_path →
  simulate end-to-end on an L-shaped outline; deterministic replay (same ops →
  same states). Outcome: 4 tests (pipeline, simulated walk, bitwise replay,
  error propagation); simplify-check promoted the shared L-mesh and agent
  helpers into `tests/test_util.hpp` (deduplicating three files) and added the
  missing SIM-009 terminal-stability step after arrival; 98/98 green.
- [x] **S5.2 Full quality pass.** `-Wall -Wextra -Wpedantic` warning-clean;
  clang-format clean (add `.clang-format` if absent); review-worker pass on
  all sources; doc-check: architecture.md + this plan match the code; README
  untouched contract respected (public headers unchanged except private
  additions — verify with diff against supplied versions).
  - **Outcome (S5.2):** Added `.clang-format` (root) plus
    `include/vwmini/.clang-format` (`DisableFormat`), normalized `src/`+`tests/`;
    public headers verified byte-identical to supplied contract f998008. Clean
    rebuild warning-free under `-Wall -Wextra -Wpedantic`; clang-format clean;
    100/100 tests pass. The review-worker pass surfaced 2 MAJOR + 4 MINOR +
    3 NIT findings: all MAJOR/MINOR fixed, the 3 NITs rejected/deferred (see
    WP5 review note).
- [x] **S5.3 Final verification & commits.** Full clean build, ctest green,
  Conventional-Commits history squashed into logical units.
  - **Outcome (S5.3):** A fresh out-of-tree configure + Release build is
    warning-clean under `-Wall -Wextra -Wpedantic`; 100/100 tests pass;
    clang-format clean; public headers byte-identical to f998008; no tracked
    build artifacts (`build/`, `.cache/`, `compile_commands.json` gitignored);
    no debug artifacts. History is already one logical unit per Conventional
    Commit (chore → feat×5 → test → style → fix → docs) with no WIP/fixup
    noise, so no squashing was required. **The implementation plan (WP0–WP5)
    is complete.**

## Revisions

Deviations from the plan above, recorded newest-first with their evidence:

- **WP5 revision (review-fix, S5.2):** The review-worker pass surfaced findings
  that were triaged against the frozen-contract and determinism constraints.
  Evidence: all fixes verified by a clean rebuild, clang-format clean, and
  100/100 tests (including new regressions).
  - **MAJOR — detour corner-cutting (SIM-012).** The detour acceptance gate
    checked only the trial endpoint against the mesh, so a straight detour step
    across a concave notch could cut through unmeshed space (reproduced: agent
    at (1.08,1.08) after a step on an L-notch mesh). Fixed by requiring the
    *whole* desired segment `[position, position+desired*dt]` to be covered,
    via the public `find_path` as a coverage oracle (`segment_in_mesh`:
    `find_path` succeeds with exactly two points, which SIM-002's endpoint
    preservation makes equal to the segment's endpoints); otherwise the agent
    route-walks (which respects containment). The reviewer's suggested shared
    `segment_covered` header was **rejected**: the frozen `nav_mesh.hpp`
    friends only `find_path`, so `simulation.cpp` cannot reach
    `NavMesh::m_impl`. Regression:
    `SimulationAvoidance.ConcaveCornerAvoidanceStaysContained`.
  - **MAJOR — moved-from `NavMesh` null deref.** Both `NavMesh` instance
    methods (`contains`, `cell_count`) and the friend `find_path` — the only
    consumers of `m_impl` — now guard null, so a moved-from mesh behaves as
    empty (`false`/`0`/`OutsideMesh`), mirroring the `Simulation` moved-from
    guards. Regression: `NavMesh.MovedFromMeshBehavesAsEmpty`.
  - **MINOR — `orient` double re-widening.** Fixed `internal::orient` to widen
    each coordinate to double *before* subtracting (it previously re-widened
    float-rounded differences), matching `triarea2` and the architecture's
    double-cross-product specification.
  - **MINOR — `apply_goal` non-transactional.** Route is now computed before
    any agent-state mutation; on the (currently unreachable) find_path failure
    the agent is left untouched.
  - **MINOR — dead `tests/` include dir** (`src/`) removed from CMake.
  - **MINOR — missing `using internal::dist`** added to simulation.cpp.
  - **Rejected/deferred:** dead `src/vwmini.cpp` stub kept (initial-contract /
    WP0 build placeholder); double-precision `desired_velocity` deferred
    (cross-module behavior change; the float pipeline is the documented, tested
    contract); `PlanScore` field alignment is cosmetic.
- **WP5 revision (style, S5.2):** Superseded the WP4 guidance to accept manual
  wrap in existing files rather than reformat repo-wide. Evidence: with the
  derived `.clang-format` in place the measured reformat was ~75 changed lines
  across 11 files (purely mechanical wrapping/braces), and the bitwise-exact
  `Integration.DeterministicReplay` test still passes, proving behavior is
  unchanged. Normalizing everything makes "clang-format clean" a verifiable
  gate instead of a judgement call. `include/vwmini/` is excluded via a local
  `DisableFormat` file so the public headers stay byte-identical to the
  supplied contract (verified: zero diff against f998008).
- **WP4 revision (style, for S5.2):** No `.clang-format` exists and the
  committed sources are not reproducible by any single config (manual wrap
  choices). The de-facto style, derived by diffing committed files against
  clang-format: LLVM base, `IndentWidth: 4`, `ColumnLimit: 100`,
  `PointerAlignment: Left`, `SortUsingDeclarations: false`,
  `AllowShortFunctionsOnASingleLine: None`, Custom brace wrapping —
  functions Allman; control statements/structs/namespaces attach. S5.2 should
  commit this config, apply it to new files, and accept manual-wrap judgement
  in existing files rather than force a repo-wide reformat.
- **WP3 revision (design):** No `internal/pathfinding.hpp` was needed —
  `find_path` is self-contained in `pathfinding.cpp` (anonymous namespace) and
  `Simulation` will route through the public `find_path`; only the shared
  representation helpers moved into the existing `mesh_data.hpp`.
- **WP2 revision (simplify-check):** Two correctness bugs found & fixed during
  S2.3. (1) `triangle_contains_eps` originally intersected epsilon-inflated edge
  *half-planes*, which overshoots MSH-006 near corners (accepts up to
  eps/sin(theta/2), e.g. 1.41*eps at 90 deg); replaced with the exact contract
  disjunction `closed-interior OR distance-to-any-edge-segment <= eps`.
  (2) `check_t_junctions` compared a cell against itself, wrongly rejecting valid
  thin triangles whose apex is within eps of their own base; now skips `i == j`.
  Both confirmed by new regression tests (37 total).
