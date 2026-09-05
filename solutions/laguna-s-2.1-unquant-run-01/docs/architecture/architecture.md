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
   gives exact manifold, T-junction, and overlap checks without fuzzy edge hashing,
   satisfying the "safe and deterministic" clause for ambiguous near-epsilon geometry.
3. **Funnel (string-pulling) routing**: Dijkstra over the dual graph (Euclidean
   centroid weights, deterministic tie-break by cell index) selects a cell corridor;
   the apex-based funnel then pulls the route taut through shared-edge portals. Portal
   orientation per edge uses the local centroid direction so left/right chains stay
   consistent. The result is a contained polyline; redundant points (< `epsilon` apart)
   are collapsed. Determinism is preserved; the winner among equal-cost corridors is
   not prescribed.
4. **Sub-stepped motion with snapshot-based avoidance**: `step` splits the duration
   into fixed sub-steps. Each sub-step computes desired velocities from the snapshot of
   live state (read before any write — `SIM-010`), then resolves collisions with a
   reciprocal velocity projection that zeroes the approach velocity along each contact
   normal (no teleport, no speed excess). Sub-step size bounds displacement so a single
   call cannot overshoot a waypoint; agents stop exactly on the goal corner when reached.
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
- Reciprocal velocity projection (rather than full RVO) is sufficient for the required
  small open-space two-agent acceptance (`SIM-011`) and keeps the avoidance logic
  deterministic and deadlock-free by construction.
