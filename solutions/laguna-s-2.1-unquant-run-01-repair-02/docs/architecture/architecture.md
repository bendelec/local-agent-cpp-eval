# VWmini — Internal Architecture

VWmini is a compact C++23 library for immutable 2D navigation meshes, deterministic
point-to-point paths, and local avoidance between disc agents. The public surface
(`include/vwmini/`) is fixed; this document describes the implementation chosen under
`src/`.

## Components & Responsibilities

| Component (file) | Responsibility | Owns |
|---|---|---|
| `geometry.hpp` (public) | Value-semantic `Vec2`, `Polygon`, `Error`/`Result`, `ErrorCode`, `triangulate_simple_polygon`, `length`, `normalized` | — (declarations) |
| `src/vwmini/geometry.cpp` | Definitions of `length` / `normalized` | — |
| `src/vwmini/detail.hpp` | Shared internal geometry helpers: epsilon, signed area, point-in-triangle, point/segment distance, segment intersection | — |
| `src/vwmini/triangulate.cpp` | `triangulate_simple_polygon` (ear-clipping) + validation | — |
| `src/vwmini/navmesh_detail.hpp` | Definition of `NavMesh::Impl` (private nested struct, out-of-line) | triangle vertices, adjacency |
| `src/vwmini/nav_mesh.cpp` | `NavMesh::create`/`contains`/`cell_count` + adjacency build + validation | owns `m_impl` via `shared_ptr<const Impl>` |
| `src/vwmini/path.cpp` | `find_path` (cell location → Dijkstra → funnel) | borrows `NavMesh::Impl` |
| `src/vwmini/simulation.cpp` | `Simulation::Impl`: agent table, routes, goal state, sub-stepped motion + local avoidance | owns `unique_ptr<Impl>` |
| `tests/` | Deterministic GTest checks | — |

## Dependency & Ownership Flow

```
        ┌─────────────┐
        │  NavMesh    │  immutable value, shared via shared_ptr<const Impl>
        └──────┬──────┘
               │ const ref
        find_path()  ──► (funnel uses mesh triangles + adjacency)
               │
        ┌──────┴──────┐
        │  Simulation │ owns NavMesh by value; owns agent table
        └─────────────┘
```

`NavMesh` is copyable and immutable once `create` accepts it (a `shared_ptr<const Impl>`).
`Simulation` takes the `NavMesh` by value (copies in the mesh) and is move-only. No
component writes to a mesh after construction. `find_path` is a friend of `NavMesh` and
reads `m_impl` directly; it is the only other code that needs `NavMesh::Impl`.

## Key Design Decisions

1. **Ear-clipping triangulation** (`triangulate_simple_polygon`): simple, deterministic,
   O(n²) validation. Collinear tips are skipped so they never yield degenerate ears;
   the first valid ear in index order is always chosen (deterministic output order).
2. **Explicit, small-mesh-oriented validation**: O(T²) edge pairing in `NavMesh::create`
   enforces only the `MSH-004`/`MSH-005` rejections that indicate genuine geometry
   defects — non-manifold **edges** (an edge used by more than one other triangle),
   **T-junctions** (a triangle vertex lying strictly in the interior of another
   triangle's edge), and **overlapping interiors** (a proper crossing of non-adjacent
   edges, a same-side fold across a shared edge, or a vertex strictly inside the
   opposing triangle). It deliberately does **not** reject per-vertex fan configurations:
   a vertex touching another triangle's edge only at an endpoint is a valid vertex-only
   contact that creates no adjacency (`MSH-005`), and disjoint valid components are
   allowed. Near-epsilon edge-endpoint matching uses `epsilon`; anything closer is
   numerically ambiguous and may be accepted or rejected, but always deterministically.
3. **Funnel (string-pulling) routing**: Dijkstra over the dual graph (Euclidean
   centroid weights, deterministic tie-break by cell index) selects a cell corridor;
   the apex-based funnel then pulls the route taut through shared-edge portals. The
   funnel is seeded with a degenerate start portal (the start point) and terminated with
   a degenerate goal portal (the goal point). Appending the goal as a zero-length portal
   makes reflex corners along the corridor boundary pinch the apex onto the boundary
   instead of emitting an unconditional `[start, goal]` straight segment that could
   leave the mesh (the L-shaped corridor case). Portal orientation per edge uses the
   local centroid direction so left/right chains stay consistent, and cone narrowing
   walks each leg toward the more-interior endpoint of the incoming portal. Vertex
   more-interior endpoint of the incoming portal. The result is a
   contained polyline whose consecutive points differ by at least `epsilon`.
   Determinism is preserved; the winner among equal-cost corridors is not prescribed.
4. **Sub-stepped motion with snapshot-based, candidate-selection avoidance**: `step`
   splits the duration into fixed sub-steps. The sub-step *count* is bounded: `ratio =
   seconds / target_substep` (safe because `seconds` is validated finite-and-positive
   before the division), then `n_sub = min(ceil(ratio), max_substeps)` with
   `max_substeps = 100000` and `sub_dt = seconds / n_sub`, so a huge-but-finite duration
   (`SIM-008`) cannot overflow `float` or turn `ceil → size_t` into undefined behaviour.
   Each sub-step reads a snapshot of live state (position, velocity, destination
   waypoint) before any write — `SIM-010` — then advances with candidate-selection
   local avoidance (no teleport, no speed excess, no state outside the mesh). For each
   moving agent a bounded, deterministic candidate set is built and mesh-clamped in
   place: the goal-directed velocity (`c0`, arrival-capped) plus yield/stop/strafe/corner
   candidates, and — for each proximate neighbour — a reciprocal separating push along
   the contact normal, a full normal separation, both tangential sidesteps, and
   reciprocal half/full yields when approaching. Agents then best-respond to their
   neighbours' current choices in an 8-round Gauss-Seidel sweep (earlier indices commit
   first, which breaks coincident-centre symmetry) ranked by a tuple key:
   collision-free first, then least penetration, then most progress toward the waypoint,
   then most end-of-step separation, then a stable index. This lets a pair of crossing
   agents slide past each other (tangential sidestep) and resolve a circumstantial
   overlap (contact-normal push) while staying goal-directed, so overlapping agents
   separate without goal-cancelling jitter or non-finite state (`SIM-010`/`SIM-012`).
   Sub-step size bounds displacement so a single call cannot overshoot a waypoint; agents
   stop exactly on the goal corner when reached.
5. **Agent table as monotonic id → dense vector with tombstones**: `AgentId` is a plain
   `uint32` with no generation, so removed ids are kept as tombstones (never reused) to
   guarantee `NotFound` for unknown/removed ids while keeping `agent()` snapshot values.
6. **No global state, no I/O, no threads**: all state lives inside the value objects;
   `contains` is const and allocation-free; error paths never mutate accepted meshes.

## Interfaces (internal seams)

- `detail.hpp` exposes pure functions operating on `Vec2`/`Polygon` only — no
  coupling to mesh/agent state.
- `NavMesh::Impl` is the single seam between mesh construction and pathfinding; only
  `find_path` (friend) crosses it.
- `Simulation::Impl` owns the mesh copy and the agent table; its `step`/`avoidance`
  logic is self-contained.

## Trade-offs

- Correctness and testability over micro-optimization. Algorithms are O(n²) or O(T²),
  acceptable because nav-meshes here are small authored inputs and the task sets no
  performance target (`NFR-008`).
- A heuristic corridor selector (Dijkstra) plus exact funnel is simpler and more
  verifiable than implementing globally-optimal A* with a funnel-consistent metric, while
  still satisfying the "funnel/string-pulling" and determinism contracts.
- Candidate-selection avoidance with an iterative best-response selector (rather than
  full RVO) is sufficient for the required small open-space two-agent acceptance
  (`SIM-011`) and keeps the avoidance logic deterministic (fixed candidate set, index
  order) and deadlock-free by construction.
