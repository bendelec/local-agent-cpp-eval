# VWmini Architecture

VWmini is a single-header-public, multi-source C++23 library. Public headers under
`include/vwmini/` fix the API; all implementation lives under `src/`. Only the C++
standard library is used. The library is immutable-by-construction after successful
construction, value-semantic, exception-free at its public surface (`Result<T>`), and
single-threaded.

## Components and responsibilities

| Component | Files | Responsibility |
|---|---|---|
| **Geometry** | `geometry.hpp`, `src/geometry.cpp` | `Vec2` ops, `length`, `normalized`; numeric helpers (`cross`/`dot`/`epsilon`). Pure functions only. |
| **Triangulator** | `src/triangulator.cpp` | Validate one simple CCW hole-free outline and produce deterministic ear-clipping triangles. |
| **NavMesh** | `nav_mesh.hpp`, `src/nav_mesh.cpp` | Immutable mesh storage, validation, adjacency derivation, `contains`, `cell_count`. Owns an opaque `shared_ptr<const Impl>`. |
| **Pathfinder** | `src/path_finder.cpp` | `find_path`: endpoint validation, visibility-graph Dijkstra, deterministic shortest contained route. Friend of `NavMesh`. |
| **Simulation** | `simulation.hpp`, `src/simulation.cpp` | Agent lifecycle, routes, stepping, local disc avoidance. Pimpl owns agents + mesh copy. |

Public types stay value-semantic; internals are hidden behind pImpl so the ABI and
representation are unobservable (MSH-008).

## Dependency / data flow

```
   geometry.hpp  <-------------------  (value types, no deps)
        ^                              ^
        | includes                    | includes
triangulator.cpp          nav_mesh.cpp / path_finder.cpp
   |                            |            |
   v                            v            v
triangulate_*            NavMesh(create)    find_path(mesh,...)
                                ^                ^
                                |                | friend
                                +------- Simulation(owns NavMesh copy)
                                          add_agent/set_goal/step/agent...
```

Direction of dependency is strictly inward toward the standard library. Nothing depends
upward on `Simulation`. `find_path` reads `NavMesh` through friendship rather than a
public accessor, preserving representation opacity while keeping pathfinding decoupled
from simulation state.

Ownership: `NavMesh::create` builds a mutable working structure then wraps it in
`std::shared_ptr<const Impl>`; copies share ownership of identical immutable data.
`Simulation` holds its own `NavMesh` value plus a `unique_ptr<Impl>` for agent state.
No raw owning pointers exist anywhere.

## Important seams and interfaces

- **`NavMesh::Impl`** (private, in `nav_mesh.cpp`): stores triangle vertex array, per-cell
  plane equations / edge normals, adjacency index table, and optional AABBs. Exposes only
  what `contains` and `find_path` need. Because it is held as `shared_ptr<const Impl>`,
  const operations remain allocation-free and thread-safe-in-practice (not contracted).
- **`SegmentCover`**: a small private helper (in `path_finder.cpp`) that tests whether a
  segment lies inside the union of closed triangles. It is reused both for direct-path
  detection and for visibility-graph edge construction, guaranteeing the two notions agree.
- **`Agent`/`Route`** (private, in `simulation.cpp`): each live agent stores position,
  velocity, radius, max speed, goal, effective arrival radius, status, current waypoint
  index, and the `Path` polyline returned by `find_path`. No external mutation reaches them.

## Major decisions and trade-offs

1. **Visibility graph + Dijkstra instead of cell corridor funnel.** The Euclidean shortest
   path in this polygonal domain bends only at triangle vertices, so building a visibility
   graph over `{start, goal} ∪ {triangle vertices}` with edges = segments provably contained
   in the mesh yields the *globally* shortest contained route. This makes SIM-003/SIM-004
   exact and removes hand-tuned funnel orientation bugs. Cost is O(V²·T) per query — fine for
   test-scale meshes; bbox prefiltering keeps it responsive. Determinism comes from ascending
   neighbor-index relaxation order. Direct visibility falls out naturally as the `[start,goal]`
   edge, satisfying SIM-001 exactly without special-casing beyond equal-endpoint handling.

2. **Ear clipping triangulation.** Deterministic, simple, handles permitted collinear
   vertices by skipping zero-area ears while still removing redundant vertices. Output
   ordering is fixed by always scanning low-to-high indices.

3. **Single-file self-contained test framework.** To avoid any external test dependency
   (robustness across grader environments), `tests/testing.hpp` provides minimal assertion
   macros; each focused test file exposes `int run_..._tests()` called in explicit order from
   a central main, giving fully deterministic execution independent of static-init order.

4. **Local avoidance via snapshot-then-update steering.** Per substep, all desired velocities
   are computed from a frozen snapshot of positions/velocities, mutual repulsion is applied
   symmetrically, speeds are clamped to `max_speed`, motion is integrated with waypoint/goal
   overshoot clamping and mesh projection, and residual overlaps receive symmetric position
   correction re-projected into the mesh. This satisfies SIM-010–SIM-012 deterministically
   and keeps two crossing open-space agents apart within tolerance.

5. **Tolerance discipline.** `epsilon = 1e-4f` used only where MSH/SIM explicitly require it
   (edge matching, containment distance, consecutive-point dedup, interval merging). All
   other comparisons use exact float equality (`Vec2::operator==`).

## Validation strategy

Every public entry point validates inputs first and returns the documented `Error` before
mutating observable state; transactional creation means a failed `NavMesh::create` or
`step` leaves nothing partially constructed. Tests assert both happy paths and every error
code branch listed in NFR-006.
