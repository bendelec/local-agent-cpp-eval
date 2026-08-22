# VWmini Architecture

## Overview

VWmini is a single C++23 static library implementing an immutable 2D navigation
mesh, deterministic point-to-point paths, and local disc-agent avoidance. It is
deliberately small: six internal modules with strictly inward dependencies,
value semantics throughout, and `std::expected`-based error reporting at every
fallible public boundary. There is no global state, no allocation on `contains`,
and no threads (NFR-001, NFR-003, NFR-008).

## Module Diagram

```mermaid
flowchart TD
    subgraph public["Public API (include/vwmini)"]
        G[geometry.hpp] --> N[nav_mesh.hpp] --> S[simulation.hpp]
    end
    subgraph internal["Internal (src/vwmini)"]
        GP[geometry_priv.hpp] --> T[triangulate.{hpp,cpp}]
        GP --> M[mesh_priv.{hpp,cpp} + nav_mesh.cpp]
        GP --> P[path.{hpp,cpp}]
        M --> P
        GP --> SI[simulation.cpp + simulation_impl.hpp<br/>(incl. local disc avoidance)]
        M --> SI
        P --> SI
    end
    G -. uses .-> GP
    N -. uses .-> M
    S -. uses .-> SI
```

Dependencies are acyclic and point inward: `geometry_priv` ←
{triangulate, mesh_priv, path, simulation}; `mesh_priv` ← {path, simulation};
`path` ← simulation. Local disc avoidance is a function group inside
`simulation.cpp`, not a separate module (NFR-008: no speculative extension
layers). Nothing internal depends on the public simulation header.

## Modules

| Module | Responsibility |
|---|---|
| `geometry_priv` | Finite predicates, Vec2 finiteness (`is_finite`), signed area/orientation, point-to-segment distance, `Polygon`/triangle validation. Pure, no state. |
| `triangulate` | `triangulate_simple_polygon`: full validation (MSH-002) + ear-clipping. Returns `Result<std::vector<Polygon>>`. |
| `mesh_priv` | `NavMesh::Impl`: validates triangles (MSH-004), builds shared-edge adjacency (MSH-005), and answers `contains` by scanning all cells (MSH-006). Immutable after construction; `NavMesh` holds `shared_ptr<const Impl>` so copies are cheap value copies (public-api "value and lifetime contract"). |
| `path` | `find_path` (SIM-001…004): endpoint validation, exact direct-containment shortcut, Dijkstra over cell adjacency, iterative string-pulling (coordinate descent over the portal crossing points — funnel-equivalent shortening) with endpoint snapping, polyline refinement (point-omission), and an exact per-segment containment gate (SIM-002 "every real t"). |
| `simulation` | Public `Simulation` PIMPL (SIM-005…013): agent storage and id lifecycle, goal state machine, deterministic substep stepping (snapshot → steer → avoid → integrate → clamp → transitions). Local disc avoidance (SIM-010…012) is implemented here: per-substep snapshot of all agents (positions + pre-avoidance steer velocities), waypoint steering clamped by max speed with exact waypoint landing (no overshoot), RVO velocity-obstacle projection (double cone in relative-velocity space; deterministic anti-symmetric fallback for coincident centres), direct centre-line repulsion inside the clearance radius (degenerate-VO zone, see below), single integrated move with post-move containment clamp (48-iteration bisection). Route indices advance ONLY on exact waypoint landing (never on mere proximity), so a consumed waypoint is always one the agent stands on, from where the path-validated next segment is walkable. Constants: `kAvoidLookahead = 1.5` m, `kAvoidMargin = 0.4` m, `kAvoidWeight = 1.0`, `kAvoidSeparationBoost = 2.0`. |

The public `Simulation` (header-supplied, PIMPL) delegates everything to a
`Simulation::Impl` that owns `std::vector<SimAgent>` storage (slot i ==
AgentId{i+1}; ids are non-zero `uint32_t`, monotonically increasing, no reuse —
SIM-013 invalidation is trivially satisfied).

## Key Seams

- **`contains` contract (MSH-006)**: non-finite → false; strict interior of any
  triangle → true; within `epsilon` of any triangle **edge segment** → true;
  otherwise false. Verified in tests at sub-pixel tolerances.
- **Path verification gate**: the path builder refines the pulled polyline
  (omitting consecutive points closer than `epsilon`, or collinear ones), then
  *checks* every output segment with an exact MSH-006 interval test — for each
  triangle it computes the set of parameter `t` where the segment point is in
  the strict interior or within `epsilon` of an edge segment (as interval
  arithmetic on `t`), merges the coverage, and requires the uncovered length to
  be ~0. On any failure it returns `NoPath` rather than a violating polyline.
  This makes SIM-002's strong "every real t" property a checked invariant,
  not an aspiration. The same test implements SIM-001's
  "straight segment is contained" shortcut.
- **Determinism**: no unordered iteration affects decisions; the cell grid and
  adjacency maps are iterated by sorted key; all math is float (no hidden
  platform variance beyond the platform clause). `find_path` on identical input
  produces bit-identical point sequences.
- **Transactional setup**: `NavMesh::create` and `Simulation::add_agent` build
  into locals and commit only after full validation (MSH-007, SIM-008 edge case
  for `step`'s invalid input).

## Major Decisions

1. **Ear clipping (no incremental triangulation).** MSH-002 restricts input to
   simple, CCW, hole-free polygons. Ear clipping with a full overlap check is
   simple, deterministic, and adequate for the stated scope (NFR-008 forbids
   scope growth). Worst case O(n²) — acceptable for authoring-time use.
2. **Mesh validation is strict**: exact triangle (3 verts), non-degenerate
   (|signed double area| > ε²), CCW, no interior overlap (segment intersection
   on non-adjacent edges, plus T-junction rejection via point-on-segment with
   endpoints matching to ε), no non-manifold edges (an edge used by >2
   triangles). Overlap of interiors of *adjacent* triangles sharing an edge is
   inherently excluded by the edge-sharing construction.
3. **Contains = strict-interior OR near-edge** (MSH-006, verbatim). This admits
   points slightly outside the triangle — intentional per the spec's edge
   cases.
4. **Routing = Dijkstra + string-pulling over shared-edge portals** (SIM-003).
   Cell adjacency graph is built from exact shared edges (MSH-005, endpoints ≤
   ε). A* is unnecessary: Dijkstra with edge length over a planar mesh gives a
   shortest route; ties are broken deterministically by (distance, cell index).
   The corridor is then shortened by iterative string-pulling: each portal
   crossing point minimises its two-neighbour block cost over the portal
   segment (endpoints or the line crossing), a jointly convex problem, so
   coordinate-wise optimality is a global minimum — this is the
   funnel/string-pulling result the spec requires (no unshortened
   cell-centre route). Consecutive portals share one convex cell, so the
   pulled polyline stays inside the corridor by construction; pulled points
   within 1e-5 m of a portal endpoint snap to the exact vertex, keeping
   output corners exact. Endpoints on shared boundaries make several start/goal
   cells candidates; all are tried in index order and the shortest verified
   route wins (deterministic).
5. **Avoidance = RVO velocity-obstacle projection, single-substep** (SIM-010,
   SIM-011, SIM-012). Each substep takes a snapshot of all live agent
   positions and velocities; each MOVING agent then computes a conflict-free
   velocity by projecting its waypoint steer velocity onto the nearest point
   outside the RVO double cone (apex at `-rel/t`, half-angle `asin(R/dist)`)
   for every other agent whose predicted closest approach falls inside
   `kAvoidLookahead`. The final velocity is clamped to `max_speed`; a
   containment clamp (48-iteration bisection) is a safety net (SIM-012).
   Two degenerate regimes get direct handling (both found as review-time
   defects, both regression-tested):
   - **Overlapping discs** (`dist < r_a + r_b`): pushed apart along the
     centre line with a push of `max_speed * ((r - dist)/r +
     kAvoidSeparationBoost)`, boost = 2.0. The push must strictly dominate
     the worst-case mutual approach (`ms_a + ms_b`, both steering at each
     other at full speed); boost 2.0 gives a relative separation rate
     `S >= ms_a + ms_b > 0` for every overlap depth, so the pair separates
     in finite time (any boost <= 1 leaves a stable contact/overlap
     equilibrium). Deterministic anti-symmetric fallback (smaller slot id
     pushes +x) when centres coincide.
   - **Inside the clearance radius, not overlapping** (`r_a + r_b <=
     dist < R`): the velocity obstacle covers the ENTIRE feasible set —
     the pair is already closer than the clearance, so no velocity keeps
     the predicted closest approach >= R and the cone projection is
     degenerate (half-angle >= 90°, `w_boundary == 0`). The agent repels
     directly away from the other agent by `2 * max_speed` (guaranteed
     strictly positive separation rate `S >= ms_a + ms_b > 0` for any
     speed combination); the pair leaves the band and the well-defined
     cone — or a lateral pass — takes over.
   Substep duration is fixed (10 Hz internal clock, capped so total steps
   bound runtime deterministically).
6. **PIMPL with `shared_ptr<const Impl>` for `NavMesh`** — the supplied header
   fixes this shape; it also gives the "copying an accepted mesh preserves its
   immutable triangles" contract at O(1).
