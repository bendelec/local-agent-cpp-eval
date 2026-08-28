# VWmini Architecture

## Overview

VWmini is a compact, single-threaded C++23 library with three public areas: a
geometry helper (`triangulate_simple_polygon`), an immutable triangle mesh
(`NavMesh`), and a disc-agent simulation (`Simulation`). The library is organized as
four small implementation modules — `geometry`, `nav_mesh`, `path`, and `simulation` —
that share a private header of pure geometric predicates. Dependencies flow inward:
no module depends on another module's internals; `path` and `simulation` depend only
on the `NavMesh` public contract plus the shared predicate header. The public headers
in `include/vwmini/` are the fixed contract and are not modified except for pre-existing
private members.

## Module Diagram

```
                    ┌──────────────────────┐
                    │  include/vwmini/*.hpp │  public contract (fixed)
                    └──────────┬───────────┘
                               │ uses
        ┌───────────────┬──────┴───────┬───────────────────┐
        ▼               ▼              ▼                   ▼
┌───────────────┐ ┌──────────────┐ ┌──────────────┐ ┌───────────────┐
│  geometry     │ │  nav_mesh    │ │  path        │ │  simulation   │
│  (triangulate)│ │  (validate,  │ │  (direct-walk│ │  (agents,     │
│  Vec2 helpers │ │   contains)  │ │   A*, funnel)│ │   stepping,   │
└──────┬────────┘ └──────┬───────┘ └──────┬───────┘ │   avoidance)  │
       │                 │                │         └───────┬───────┘
       └─────────────────┴── predicates ──┴─────────────────┘
                          ┌──────────────────┐
                          │ internal/        │  pure functions,
                          │ geometry_detail  │  no state
                          └──────────────────┘
```

## Modules

### geometry (`src/vwmini/geometry.cpp`)
- **Responsibility**: `Vec2::length`/`normalized`; validate one simple CCW hole-free
  outline and triangulate it with deterministic ear clipping (MSH-001–MSH-003).
- **Ownership/data flow**: reads a `const Polygon&`; returns a `Result<std::vector<Polygon>>`
  of owned CCW triangles. No state.
- **Key contracts**: non-finite coordinate → `InvalidArgument`; structural invalidity
  (fewer than 3 vertices, cyclic duplicates, self-intersection, clockwise/degenerate
  winding) → `InvalidMesh`; output preserves area within
  `epsilon * max(1, area)`; identical input → identical ordered output.

### nav_mesh (`src/vwmini/nav_mesh.cpp`, `nav_mesh_detail.hpp`)
- **Responsibility**: validate a non-empty triangle list (MSH-004), build adjacency from
  matched complete edges (MSH-005), expose `contains` (MSH-006) and `cell_count`.
- **Ownership/data flow**: `NavMesh::Impl` is a `shared_ptr<const Impl>` (declared in the
  supplied header); the definition lives in a private header so `path` can read it.
  `Impl` owns the triangles and precomputed neighbour/boundary tables. `NavMesh` is a
  regular value type; creation is transactional (validation completes before any object
  exists).
- **Key contracts**: all validation before construction; accepted meshes are immutable;
  `contains` is `const noexcept` and allocation-free; patterns inside the `epsilon`
  ambiguity band are handled deterministically and are documented as outside the
  interoperability guarantee. A vertex on another triangle's open edge within
  `epsilon` (a T-junction) is rejected as `InvalidMesh`; complete shared edges and
  point-touch contacts remain valid.

### path (`src/vwmini/path.cpp`)
- **Responsibility**: `find_path` — endpoint validation, exact direct-segment
  containment test, A* corridor search over the triangle adjacency graph, and
  Simple-Stupid-Funnel string pulling through the corridor portals (SIM-001–SIM-003).
- **Ownership/data flow**: reads the `NavMesh` (via the `find_path` friend) and returns
  an owned `Path`. No state.
- **Key contracts**: `InvalidArgument`/`OutsideMesh`/`NoPath` diagnostics per SIM-001;
  deterministic; funnel-shortened polyline; verified segment containment with a
  deterministic portal-midpoint fallback for epsilon-band corner cases. The
  containment walker keeps a monotone `t_walk` (events strictly advance; no
  visited-cell guard, no ping-pong), treats boundary crossings as continuations
  while the ε/2 probe ahead stays in the band, and runs under an explicit
  iteration budget so band endpoints yield exact direct paths, never `NoPath`.

### simulation (`src/vwmini/simulation.cpp`)
- **Responsibility**: agent lifecycle (`add_agent`, `remove_agent`, `set_goal`,
  `clear_goal`), snapshot queries, and deterministic `step` with substep integration and
  local pairwise avoidance (SIM-005–SIM-013).
- **Ownership/data flow**: `Simulation::Impl` (defined in the `.cpp`) owns a `NavMesh`
  value and a `std::vector<std::optional<Agent>>` indexed by `id - 1`; removal leaves a
  tombstone so ids stay valid-opaque and iteration order is id order. `agent()` returns
  a value snapshot; nothing internal is exposed.
- **Key contracts**: validation of every setup/mutation call with the exact
  `Result`/`Error` taxonomy; `step` validates before mutating (transactional);
  substep loop reads a position/velocity snapshot of all agents before any update;
  velocities clamped to `max_speed`; positions clamped into the mesh with wall
  sliding (a blocked move keeps its tangential component along the boundary, from
  a rotation-invariant containment probe fan); arrival radius rule with `-1.0f`
  sentinel. The substep count is computed in `double` and capped at
  `kMaxSubsteps = 65536` before the `size_t` conversion, so every finite
  non-negative duration steps in bounded time; the loop exits early once no agent
  is `Moving`. Same-lane pairs (heading dot > 0.25) split sides anti-symmetrically
  (follower right, leader left), pinned per encounter, so an overtaking pair keeps
  at least `r_a + r_b` of separation.

## Shared predicates (`src/vwmini/internal/geometry_detail.hpp`)

Pure, allocation-free, stateless inline helpers: finiteness, double-precision signed
area / orientation, point-to-segment distance, segment intersection, strict and
tolerance-aware point-in-triangle. All geometric predicates use `double` internally for
stability; the public API stays `float`. `epsilon = 1e-4f` is applied exactly where the
requirements say it is (point/edge distance for containment, edge-endpoint matching for
adjacency, triangle degeneracy at `epsilon * epsilon`).

## Ownership and data flow

- No global mutable state; no raw owning pointers; no threads; no console output.
- `NavMesh` is returned by value from `create` (`shared_ptr<const Impl>`), copied around
  as a regular value; the pointee is immutable by construction.
- `Simulation` is move-only (per the fixed header) and owns a `NavMesh` copy.
- Agents own their route (`Path`) plus a waypoint cursor; snapping a snapshot of
  `AgentState` cannot affect the simulation.

## Important seams and interfaces

- **Predicate seam**: all geometry decisions funnel through `geometry_detail`, so
  tolerances, finiteness rules, and orientation conventions are defined once and tested
  once.
- **Adjacency seam**: `NavMesh::Impl` precomputes neighbour tables once; both the
  direct-segment walker and A* consume them without re-deriving geometry.
- **Avoidance seam**: decision logic lives in one deterministic
  `advance(agents, mesh, dt)` substep function inside `simulation.cpp`, kept separate
  from state mutation (apply phase) and from input validation (public API).
- **Containment verification**: `find_path` verifies every returned segment with the
  same walker used for the direct-path decision, so SIM-001/SIM-002 share one mechanism.

## Major decisions and trade-offs

| Decision | Rationale |
|---|---|
| Ear clipping for triangulation | Simple, deterministic, exact output area; O(n²) is fine for authoring-size outlines (NFR-008). |
| Double precision inside predicates | Avoids float round-off misclassification of orientation/touch while preserving the float API contract. |
| `epsilon` applied only where required | Prevents tolerance creep from weakening validation; ambiguous epsilon-band geometry is accepted/rejected consistently and documented. |
| Dijkstra over triangle adjacency + funnel smoothing | Uniform-cost search with a (cost, cell-index) tie-break is deterministic and canonical; the corridor is optimal w.r.t. a centroid objective; the funnel gives the required string-pulling (SIM-003). A* was deliberately not used: no heuristic benefit for small deterministic meshes. |
| Direct-segment test = triangle walk + band guards | Exact for non-ambiguous geometry; epsilon-band vertex cases resolve via deterministic probes and a documented repair path. |
| Agent storage = vector of optionals indexed by id | Deterministic iteration in id order, O(1) lookup, no id reuse; move-only Simulation avoids external map dependencies. |
| Fixed substep, count computed in `double` and capped at 65536 | Deterministic total integration time, bounded per-step motion, waypoint/goal clamping prevents overshoot (SIM-008); the cap makes huge finite durations (`1e9`, `3.4e38`) terminate instantly and removes the float→`size_t` INF overflow (UB). |
| Blended pairwise avoidance on a pre-update snapshot, anti-symmetric same-lane rule | Deterministic, local, no framework: for each close pair (i < j order) a severity is derived from the predicted closest approach (and an envelope push for pairs already too close); head-on/oblique pairs steer to the right of their own heading (world-anti-symmetric for head-on), same-lane pairs (dot > 0.25) split by behind/ahead with an encounter-pinned side, so overtakes separate laterally instead of sharing one side (SIM-010–SIM-012). |
| No spatial acceleration structures | Linear scans keep `contains` allocation-free and the whole library simple; no performance target beyond supplied tests (NFR-008). |

## Threading, errors, and RAII

- Single-threaded by contract; no shared state, so const calls are trivially safe in
  the documented single-threaded use.
- All fallible public calls return `Result<T>`/`Error`; no exceptions are used as the
  public error channel (and none propagate from library code).
- RAII everywhere: `shared_ptr`/`unique_ptr` pimpls, `std::vector`/`std::optional`
  value containers, `std::expected` error values.
