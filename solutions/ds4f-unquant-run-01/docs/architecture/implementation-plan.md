# VWmini Implementation Plan

Work packages and slices, ordered and verifiable. Status is updated as work proceeds;
plan revisions are recorded explicitly at the end.

## WP0 — Contract study and environment (done)

Goal: read all requirement/API docs, confirm toolchain, inspect supplied headers.
Verification: docs read; g++ 16 / clang 22 / cmake 4.4 / clang-format 22 available;
public headers understood (fixed API, pimpl stubs, CMake target alias exists).

## WP1 — Architecture and plan documents (done)

Goal: `docs/architecture/architecture.md` (NFR-009) + this plan (NFR-010), written
before any library source edits.
Verification: both files exist and describe the design in `architecture.md`.

## WP2 — Build scaffold (done)

Goal: CMake builds one library target from explicit sources with C++23 and
`-Wall -Wextra -Wpedantic`; a minimal test executable wired into CTest.
Dependencies: WP1.
Expected files: `CMakeLists.txt` (edit), `tests/vwmini_tests.cpp` (stub),
`src/vwmini/` source files (stubs), remove `src/vwmini.cpp`.
Verification: `cmake -S . -B build && cmake --build build && ctest --test-dir build`
all green.

## WP3 — Geometry: Vec2 helpers and triangulation (done)

Goal: `length`/`normalized` (MSH-001); `triangulate_simple_polygon` validation and
deterministic ear clipping (MSH-002, MSH-003).
Dependencies: WP2.
Expected files: `src/vwmini/internal/geometry_detail.hpp`, `src/vwmini/geometry.cpp`,
`tests/test_geometry.cpp`.
Verification: unit tests for vector ops, valid outlines (square, pentagon with
collinear vertex), area conservation, determinism, and all MSH-002 invalid cases.
Simplify-check after this slice.

## WP4 — NavMesh: validation, adjacency, containment (done)

Goal: `NavMesh::create` (MSH-004, MSH-005, MSH-007, MSH-008), `contains` (MSH-006),
`cell_count`, `NavMesh::Impl` private definition shared with `path`.
Dependencies: WP3.
Expected files: `src/vwmini/nav_mesh_detail.hpp`, `src/vwmini/nav_mesh.cpp`,
`tests/test_nav_mesh.cpp`.
Verification: tests for every MSH-004 invalid case (empty, non-triangle, non-finite,
CW, degenerate, overlap, non-manifold edge, T-junction), adjacency through accepted
shared edges, boundary tolerance containment, allocation-free/const usage;
add_agent-independent mesh copy semantics.
Simplify-check after this slice.

## WP5 — Path: direct walk, Dijkstra, funnel (done)

Goal: `find_path` (SIM-001–SIM-004): endpoint validation, exact direct-segment
decision via triangle walk, A* corridor, funnel smoothing with verification and
deterministic fallback.
Dependencies: WP4 (completed).
Expected files: `src/vwmini/path.cpp`, `tests/test_path.cpp` (both final).
Verification: tests for `InvalidArgument`/`OutsideMesh`/`NoPath`, direct `[start, goal]`,
equal endpoints `[start]`, routed concave path (endpoints preserved, segments
contained, funnel shorter than portal midline route, deterministic), epsilon-omission
of consecutive close points.
Simplify-check performed.

## WP6 — Simulation: agents, goals, stepping, avoidance (done)

Goal: `Simulation` lifecycle and queries (SIM-005–SIM-007, SIM-013), `step` semantics
(SIM-008–SIM-010, SIM-012), sampled reciprocal avoidance (SIM-010, SIM-011).
Dependencies: WP5 (completed).
Expected files: `src/vwmini/simulation.cpp`, `tests/test_simulation.cpp` (both final).
Verification: tests for all validation rules (radius/speed/arrival sentinel, goals,
NotFound, OutsideMesh), goal state transitions, clear_goal, transactional zero/negative
step, max-speed bound, arrival/Reached stability, snapshot queries, removal, and a
SIM-011 crossing + overtaking acceptance scenario (progress, no overlap > 1e-3,
deterministic repeats).
Simplify-check performed.

## WP7 — Hardening, review, final verification (done)

Goal: warning-clean full build, clang-format, full test pass, architecture/doc
consistency (NFR-007, NFR-009, NFR-010); read-only review passes.
Dependencies: WP6.
Expected files: final state of all sources, docs, tests.
Verification: completed — clean-first build with `-Wall -Wextra -Wpedantic` shows
0 warnings; all 30 test cases pass; `ctest` green; clang-format applied to the whole
tree and re-verified clean; clang-tidy (analyzer/bugprone/performance) reports only
`easily-swappable-parameters` noise on symmetric geometry predicates; architecture and
plan documents updated to match the delivered design; completion report written.

## Plan revisions

- WP5 executed as Dijkstra (uniform cost, (cost, cell-index) tie-break) instead of
  A* with a heuristic: the heuristic added complexity without benefit for these
  small deterministic meshes; h=0 keeps the result canonical.
- WP6 avoidance is a sampled deterministic scheme, not strict ORCA: pairwise
  predicted-closest-approach severity, traffic-rule lateral steering (each agent
  veers to the right of its own heading), envelope push for already-close pairs,
  speed clamping, waypoint/goal distance capping, and binary containment
  restoration. Constants documented in `simulation.cpp` (kSubstepMax=0.05,
  kAvoidRange=2.0, kHorizon=2.0, kRadial=0.6, kTangential=2.5,
  kAvoidanceStrength=2.6, envelope (r_ab+0.35-d)*2.5).
- Agent storage: `vector<optional<Agent>>` slots keyed by id-1; ids are
  monotonically assigned and never reused, so removed ids stay NotFound forever.
