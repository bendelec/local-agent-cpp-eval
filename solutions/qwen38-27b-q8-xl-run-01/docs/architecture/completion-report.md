# VWmini — Completion Report

Status: **complete** — all 20 slices S1.1–S7.3 of
`implementation-plan.md` done; 72/72 tests green and warning-clean from a
clean configure on both g++ and clang++.

## Design & modules

A single C++23 static library, with strictly inward dependencies and value
semantics throughout (see `architecture.md`, kept current throughout,
including the Mermaid module diagram):

| Module | Responsibility / why this shape |
|---|---|
| `geometry_priv` | Pure predicates: finiteness, orientation/area, point-to-segment distance, strict triangle interior, segment intersection. The shared numerical core; everything else builds on it. |
| `triangulate` | `triangulate_simple_polygon` (MSH-002/003): full outline validation + deterministic ear clipping. Chosen over incremental triangulation because MSH-002 restricts input to simple CCW hole-free polygons — ear clipping with overlap checks is simpler and adequate (NFR-008 forbids scope growth). |
| `mesh_priv` + `nav_mesh.cpp` | `NavMesh::create` validation (MSH-004: CW/degenerate/overlap/T-junction/non-manifold), shared-edge adjacency (MSH-005), and `contains` (MSH-006) as a brute-force scan — const, allocation-free, and NFR-008 sets no performance target, so a spatial grid was dropped as unjustified complexity (recorded revision). `NavMesh` holds `shared_ptr<const Impl>` (shape fixed by the supplied header) for cheap value copies. |
| `path` | `find_path` (SIM-001…004): endpoint validation, direct-segment shortcut, Dijkstra over the adjacency graph with deterministic (distance, cell index) tie-break, then **iterative string-pulling** over shared-edge portals (coordinate descent; jointly convex objective ⇒ coordinate-wise optimum is the funnel/string-pulling result SIM-003 requires — no unshortened cell-centre route). Output is refined (ε-omission, corner snapping) and then gated by an **exact per-segment MSH-006 interval containment check**; any violation returns `NoPath`, making SIM-002's "every real t" a checked invariant. |
| `simulation` (+ `simulation_impl.hpp`) | Public `Simulation` PIMPL. Slot-indexed `std::vector<SimAgent>` (slot i == `AgentId{i+1}`), monotonic non-reused ids, `live_count`. Goal state machine shared by `add_agent`/`set_goal` (Moving / immediate Reached / NoPath-as-success). `step` runs a deterministic substep loop (n = clamp(llround(seconds/0.1)); equal dt): snapshot (SIM-010) → waypoint steer with exact-landing (no overshoot) → **RVO velocity-obstacle avoidance** (double cone in relative-velocity space; overlap push with deterministic anti-symmetric fallback for coincident centres) → integrate → 48-iteration bisection containment clamp (SIM-012) → post-move transitions (arrival radius, waypoint advance, route-exhausted reach). Avoidance lives inside this module rather than a separate module — no speculative extension layer. |

## Work packages & plan

All 20 slices S1.1–S7.3 in `implementation-plan.md` are complete (`[x]`),
each with goal, dependencies, files, verification, and status. Explicit
revisions (all in the revision log, none silent):

- `contains` grid index → brute-force scan (S3.3).
- Classical funnel bookkeeping → iterative string-pulling with a
  verification gate (S4.2/4.3); test-found bug in the capsule half-plane
  sign, fixed and regression-tested.
- Internal PIMPL header renamed `simulation.hpp` → `simulation_impl.hpp`
  (S7.3) to kill a same-name collision with the public header and the
  include-ordering hack it forced.
- S7.3 simplify-check worker pass: dead code removal, dedup
  (`err_code`, `strictly_inside`, `live_agent` const overload,
  `reset_motion`, `kArrivalSentinel`, single `landing` array),
  anonymous-namespace moves, and the S63 soak's re-implemented substep
  scheme replaced by an explicit 132-call sequence. Kept deliberately: the
  `kAvoidWeight` blend form (zero-sign behaviour) and the ~1-ulp safety
  clamp.

## Files changed

- `src/vwmini/`: `geometry.cpp`, `geometry_priv.{hpp,cpp}`,
  `triangulate.{hpp,cpp}`, `mesh_priv.{hpp,cpp}`, `nav_mesh.cpp`,
  `path.{hpp,cpp}`, `simulation.cpp`, `simulation_impl.hpp`
- `tests/`: `CMakeLists.txt`, `test_util.hpp`, `geometry_test.cpp`,
  `triangulate_test.cpp`, `nav_mesh_test.cpp`, `path_test.cpp`,
  `simulation_test.cpp` (72 tests)
- `CMakeLists.txt`: library sources, `vwmini::vwmini` alias, C++23,
  `-Wall -Wextra -Wpedantic`, system GTest with FetchContent fallback,
  CTest
- `docs/architecture/`: `architecture.md`, `implementation-plan.md` (plus
  `wp5-brief.md`/`wp6-brief.md` working briefs, marked superseded)
- `include/vwmini/*`: **untouched** (verified: mtimes and all
  signatures/defaults/enum values match `public-api.md`)

## Build & test commands (exact, from scratch)

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build --parallel
ctest --test-dir build --output-on-failure
# repeat with -DCMAKE_CXX_COMPILER=clang++ into build-clang/
```

Outcomes: **0 warnings/errors under `-Wall -Wextra -Wpedantic` on g++ 16.1
and clang++ 22.1; 72/72 tests passed on both.** No `.clang-format` was
supplied, so no formatting tool was added (NFR-007 requires consistent
formatting, not clang-format); the codebase follows a single
4-space/80-col style. No console output, global mutable state, or
third-party runtime dependencies in library code.

## Deliberate behavior where the contract allows a choice

- **Tie-breaking** (MSH-008/SIM-004): Dijkstra ties broken by
  (distance, cell index); equal-cost route winners are not public behavior —
  the contract says so.
- **Substep scheme**: fixed 10 Hz base with equal `dt` per call and a
  1,000,000-substep cap — "may be internally subdivided" (SIM-008) gives
  latitude; equal `dt` keeps the math bit-deterministic.
- **Avoidance algorithm**: RVO velocity-obstacle projection (SIM-012 says
  any local deterministic method is fine), with tuned, documented constants
  (`kAvoidLookahead=1.5 m`, `kAvoidMargin=0.4 m`, `kAvoidWeight=1.0`,
  `kAvoidSeparationBoost=0.7`). SIM-011 acceptance is exercised by
  deterministic tests checking separation ≥ rᵢ+rⱼ−1e-3 m, finiteness,
  containment, and speed cap at every returned step, with both agents
  reaching their goals; a 500-substep two-run soak asserts bit-identical
  `AgentState` traces.
- **Arrival snapping**: an agent entering its arrival radius (or
  exhausting its route) snaps exactly to the goal, so `Reached` states
  expose the exact goal position — a stronger, cleaner terminal state than
  the contract minimally requires.
- **Error messages**: stable, human-readable strings (only the `ErrorCode`
  is contractual).
- **One-declaration private headers** (`path.hpp`, `triangulate.hpp`) kept
  as module seams rather than inlined — flagged by review, kept by
  decision.
