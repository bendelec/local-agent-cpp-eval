# VWmini Architecture

## Overview

VWmini is a compact, single-threaded C++23 library for an immutable 2D navigation mesh,
deterministic point-to-point paths, and local avoidance between disc agents. It is
built as three small, independently reason-aboutable modules with dependencies flowing
inward: `geometry` (vector math + polygon triangulation), `navmesh` (triangle storage,
containment, adjacency, pathfinding), and `simulation` (agents, routes, stepping,
avoidance). No module has global mutable state or external runtime dependencies.

## Components and Responsibilities

| Module | Responsibility |
|---|---|
| `geometry` | `Vec2` helpers (`length`, `normalized`), and `triangulate_simple_polygon`: validate one simple CCW hole-free outline and ear-clip it into deterministic non-degenerate CCW triangles. |
| `navmesh` | `NavMesh::Impl`: owns accepted triangles (value semantics), derives shared-edge adjacency with `epsilon` tolerance, answers `contains` under the closed-edge boundary policy, and drives `find_path` (cell A* corridor + visibility-graph Dijkstra corridor shortening). |
| `simulation` | `Simulation::Impl`: owns agents and their routes; validates configuration; `set_goal`/`clear_goal` state transitions; subdivides `step` into substeps; computes simultaneous local avoidance from a snapshot; moves agents along their routes without overshooting waypoints/goal; queries return value snapshots. |

## Dependency and Ownership / Data Flow

- `navmesh` depends on `geometry` (Vec2 math). `simulation` depends on `navmesh`
  (NavMesh for containment and `find_path`). Public headers expose the fixed API;
  implementation detail lives in `src/` and private `Impl` structs referenced from the
  public headers only by pointer/`shared_ptr`.
- `NavMesh` is a regular value type: `create` validates and then stores an immutable
  `std::shared_ptr<const Impl>`; copies share the same immutable triangles.
- `Simulation` is move-only and owns a `std::unique_ptr<Impl>` holding the mesh value
  and agent storage. `AgentState`/`Path` are value snapshots; no returned reference
  observes internal storage.
- Data flow for `step`: a snapshot of all positions/velocities is taken, avoidance
  decisions are made against that snapshot, then updates are applied per substep.

## Important Seams and Interfaces

- **Validation seam**: every fallible public call returns `Result<T>`/`Error`; geometry
  validation is kept separate from normal stepping. Errors follow NFR-006 exactly
  (`InvalidArgument`, `InvalidMesh`, `OutsideMesh`, `NoPath`, `NotFound`).
- **Boundary policy seam**: `contains` is the single authority for mesh membership and
  is used consistently by path endpoints, agent positions, and goals (MSH-006).
- **Adjacency seam**: shared-edge adjacency is computed once at mesh creation with the
  documented `epsilon` endpoint tolerance (MSH-005); pathfinding uses only that graph.
- **Determinism seam**: all routing/avoidance decisions are pure functions of the mesh
  and agent state; no randomness or time-dependent ordering.

## Major Decisions and Trade-offs

- **Ear-clipping triangulation**: simple, deterministic, and matches the single simple
  polygon authoring helper; output ordering is deterministic by construction. O(n^2)
  is fine for the bounded task (NFR-008).
- **Containment by triangle + distance-to-edge**: implements MSH-006 directly. The
  closed-segment distance check admits boundary points just outside a triangle, which
  is intentional per the boundary-tolerance edge case.
- **A* over cells + visibility-graph Dijkstra corridor shortening**: A* gives a
  deterministic corridor across shared-edge portals; a visibility graph over corridor
  vertices plus Dijkstra then produces the exact shortest polyline, so the result is
  funnel/string-pulling-equivalent and not an unshortened cell-centre route (SIM-003).
  The caller's start/goal are preserved. This was chosen over the textbook funnel
  algorithm to avoid the risk of subtle bugs while meeting SIM-003.
- **Substep loop with snapshot avoidance**: a fixed substep duration bounds per-step
  motion so agents never overshoot waypoints/goal and never exceed `max_speed`
  (SIM-008/SIM-012). Simultaneous local avoidance (SIM-010) is computed against a
  snapshot so decisions are not order-dependent.
- **Reciprocal swept-segment avoidance over a lookahead horizon**: each agent evaluates
  candidate velocity offsets by sweeping its own candidate displacement segment against
  the other agents' snapshot discs and swept segments (SIM-010/SIM-011). A short
  lookahead horizon (`kHorizon`) lets agents start a lateral swerve early enough to
  diverge, and the candidate order prefers large lateral swerves (90°/-90° before
  smaller diagonals) so head-on/crossing agents split symmetrically and
  deterministically. When no collision-free candidate meets the strict safety margin,
  an agent stops (`{0,0}`), which SIM-012 permits — the symmetric head-on face-off can
  leave agents boxed, and the crossing case exercises real passing + goal reach.
- **Shared immutable `Impl` for `NavMesh`**: copying a mesh is cheap and preserves its
  accepted triangles; `find_path` is a free function over the const mesh (no mutation).

## Diagram

```
geometry ──► navmesh ──► simulation
   │            │            │
   │ Vec2 math  │ contains   │ routes, avoidance
   │ triangulate│ adjacency  │ substep loop
   │            │ find_path  │ agent lifecycle
   └────────────┴────────────┘
         public API: include/vwmini/*.hpp
         internal detail: src/ + private Impl structs
```

## Notes

This document is updated as the implementation evolves so it accurately describes the
submitted code.
