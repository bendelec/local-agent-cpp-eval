# VWmini architecture

## Components

- **Geometry** (`geometry.cpp`) owns finite-value checks, elementary vector operations,
  outline validation, and deterministic ear-clipping triangulation.
- **Mesh** (`nav_mesh.cpp`) validates triangle topology once, then stores immutable cells,
  edges, adjacency, and connected-component labels behind `NavMesh::Impl`.  It also
  provides private containment/segment predicates to routing and simulation.
- **Routing** (`nav_mesh.cpp`) builds a deterministic visibility graph within an
  adjacency-connected mesh component.  Its edges are accepted only when the complete
  segment is contained, giving a string-pulled (rather than cell-centre) route.
- **Simulation** (`simulation.cpp`) privately owns an immutable mesh and value-owned
  agents/routes.  It validates mutations before committing them and advances a snapshot
  with bounded, deterministic local steering.

```
Polygon input -> Geometry -> NavMesh::Impl (immutable cells + adjacency)
                                  |                 |
                             find_path          Simulation::Impl
                                                   -> agent snapshots
```

## Ownership and data flow

`NavMesh` shares a const implementation so copies remain cheap regular values.  A
`Simulation` takes one such value and uniquely owns its agent table; public query values
never expose that table.  Mesh construction first builds local candidate data and only
creates the implementation after all validation succeeds.  Path and simulation logic
only observes mesh data.

## Important seams

The mesh module centralizes `contains` and complete-segment containment, so direct
paths, visibility edges, and movement use the same boundary policy.  Adjacency is built
only from complete matched edges and supplies component membership, preventing routes
through vertex-only contacts.  Simulation asks the public-equivalent path routine for
new goals and treats routes as private waypoint vectors.

## Decisions and trade-offs

For this compact, correctness-focused library, routing uses an all-visible mesh-vertex
Dijkstra graph rather than a separate funnel implementation.  It is deterministic,
returns a shortest polyline over the usual string-pulling visibility vertices, and
allows every chosen segment to be explicitly checked against the mesh.  This trades
asymptotic performance for directness; no performance target requires a spatial index.

Avoidance uses fixed small substeps and simultaneous snapshot steering: preferred route
velocity is adjusted by deterministic neighbour repulsion, then candidates are clipped
for speed, predicted disc separation, and mesh containment.  This is intentionally a
local policy, not a global crowd solver.
