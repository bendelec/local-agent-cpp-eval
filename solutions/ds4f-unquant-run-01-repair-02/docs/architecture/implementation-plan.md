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

## WP8 — Hardening round: T-junction, band coherence, step bounds, overtake, convoy (done)

Goal: repair five reported defects with general fixes (no fixture special-casing):
(1) T-junction vertices on an open edge must be rejected, not silently treated
as boundary; (2) band-contained direct paths must return exact band endpoints,
never NoPath; (3) `step` must accept ALL finite non-negative durations without
substep-count overflow or unbounded loops; (4) same-lane overtaking must keep
r_a+r_b separation; (5) close-following agents must progress through the
L-reflex corner instead of freezing at t=0.
Dependencies: WP7.
Expected files: `src/vwmini/nav_mesh.cpp`, `src/vwmini/path.cpp`,
`src/vwmini/simulation.cpp`, `tests/*`, docs.
Verification: completed — all five repro scenarios pass deterministically;
34 test cases green on GCC and Clang with `-Wall -Wextra -Wpedantic`, under
ASan+UBSan, and via CTest; clang-format clean; docs updated below.

### WP8 changes (all general, deterministic)

- **nav_mesh**: validator rejects a triangle vertex that lies on another
triangle's open edge within `epsilon` (excluding within-epsilon-of-endpoint
contacts). Complete shared edges and point-touch contacts remain valid.
- **path**: `segment_contained` walker rewritten: start cell from the band
policy (no epsilon probe that misclassified boundary events), monotone
`t_walk` bookkeeping (no `visited` guard, no ping-pong), boundary crossings
continued when the ε/2 probe ahead still finds a cell, explicit iteration
budget (`12n+128`) for guaranteed termination.
- **simulation**: substep count computed in `double` and clamped at
`kMaxSubsteps = 65536` before the `size_t` conversion (no INF/overflow UB,
no unbounded loop); early exit once no agent is `Moving` (bit-identical
no-op, keeps terminal-state stability); pairwise avoidance rewritten as an
ORCA-style velocity-obstacle cone per close pair — the escape is the nearest
cone-boundary point of the relative velocity, split symmetrically (full
escape when the partner is stationary), with a sticky per-pair escape stored
in `Impl` and released only after physical separation beyond the envelope
(no snap-back oscillation); pairs inside the envelope are always constrained
and forced apart radially at the deficit rate, so passes hold `r_ab + margin`;
the degenerate apex case (head-on charging) substitutes a deterministic
lateral escape instead of a reversal, so exactly-symmetric encounters resolve;
`lane_side` state is gone entirely (defect 3) — no traffic-side bias;
containment now wall-slides along the boundary
(rotation-invariant 36-probe fan estimates the inward normal; the tangential
component of the blocked move is kept, with bisection shrink) so agents
pressed against a wall keep progressing instead of freezing.

## Plan revisions

- WP8 avoidance replaces the WP6 scheme entirely with the ORCA-style cone
constraints described above; constants `kEnvelopeMargin`, `kEscapeGain`,
`kHorizon`, and `kMaxSubsteps` are documented in `simulation.cpp`.
- WP8 containment: binary-shrink only restores the blocked-move fraction;
wall sliding preserves tangential progress (required for the L-corridor
convoy; an agent pinned against the reflex notch otherwise freezes).
- WP8 path walker: boundary crossings are continuations, not exits — the
band containment policy is the single rule for the direct-path decision AND
final route verification.

- WP5 executed as Dijkstra (uniform cost, (cost, cell-index) tie-break) instead of
  A* with a heuristic: the heuristic added complexity without benefit for these
  small deterministic meshes; h=0 keeps the result canonical.
- WP6/WP8 avoidance is an ORCA-style velocity-obstacle scheme, not a
  severity/lateral heuristic: for each close pair on a predicted collision
  course within `kHorizon`, the cone of colliding relative velocities is built
  from the enlarged separation `r_cone = r_a + r_b + kEnvelopeMargin`; the
  escape `u` is the closest boundary point to the current relative velocity
  and each moving agent gets the half-plane `v·n ≥ low` on its own side, split
  symmetrically (full escape when the partner is stationary). The escape is
  sticky per pair: it is re-applied until the pair has physically separated
  beyond the envelope, which removes the release/re-collide oscillation that
  starved symmetric crossings. Inside the envelope the constraint is radial
  (forces the deficit `(r_cone - d)` to close per second) so passes cannot
  drift below the envelope; the apex case (relative velocity along the axis)
  uses a deterministic lateral escape instead of a reversal. There is no
  traffic-side state. Constants documented in `simulation.cpp`
  (kSubstepMax=0.05, kAvoidRange=2.0, kHorizon=2.0,
  kEnvelopeMargin=0.09, kEscapeGain=2.0).
- Agent storage: `vector<optional<Agent>>` slots keyed by id-1; ids are
  monotonically assigned and never reused, so removed ids stay NotFound forever.
