# VWmini architecture

## Components

- **Geometry** (`geometry.cpp`) owns finite-value checks, elementary vector operations,
  outline validation, and deterministic ear-clipping triangulation. Its private predicates
  widen coordinates before subtraction/products; the public finite `float` length saturates
  at `FLT_MAX` when necessary.
- **Mesh** (`nav_mesh.cpp`) validates triangle topology once, then stores immutable cells,
  edges, adjacency, and connected-component labels behind `NavMesh::Impl`. Its private
  double-coordinate predicates provide containment and complete-segment queries to routing
  and simulation.
- **Routing** (`nav_mesh.cpp`) builds a deterministic visibility graph within an
  adjacency-connected mesh component.  Its edges are accepted only when the complete
  segment is contained, giving a string-pulled (rather than cell-centre) route.
- **Simulation** (`simulation.cpp`) privately owns an immutable mesh and value-owned
  agents/routes. It validates mutations before committing them and advances one immutable
  position/velocity snapshot into deterministic, bounded local motions.

```
Polygon input -> Geometry -> NavMesh::Impl (immutable cells + adjacency)
                                  |                 |
                             find_path          Simulation::Impl
                                              snapshot -> feasible stored motions
                                                       -> committed agent snapshots
```

## Ownership and data flow

`NavMesh` shares a const implementation so copies remain cheap regular values.  A
`Simulation` takes one such value and uniquely owns its agent table; public query values
never expose that table.  Mesh construction first builds local candidate data and only
creates the implementation after all validation succeeds.  Path and simulation logic
only observes mesh data.

## Important seams

The mesh module centralizes scale-safe `contains` and complete-segment containment, so
endpoint validation, direct paths, visibility edges, and movement use the same boundary
policy. Adjacency is built only from complete matched edges and supplies component
membership, preventing routes through vertex-only contacts. Simulation asks the same
routing seam for new goals and treats routes as private waypoint vectors. Its stepping
seam explicitly separates intended velocity, mesh-feasible displacement, rounded stored
endpoint, and committed observed velocity; a clipped endpoint is committed only if a
fresh path query can still route it to the retained goal.

## Decisions and trade-offs

For this compact, correctness-focused library, routing uses an all-visible mesh-vertex
Dijkstra graph rather than a separate funnel implementation.  It is deterministic,
returns a shortest polyline over the usual string-pulling visibility vertices, and
allows every chosen segment to be explicitly checked against the mesh.  This trades
asymptotic performance for directness; no performance target requires a spatial index.

Avoidance uses fixed small substeps and simultaneous position/velocity snapshots. For
one snapshot it creates a short deterministic list of route, reduced, tangential, and
separating velocity intentions for moving agents; terminal and idle agents contribute
only stationary motions. Each moving option is independently converted to a
speed-budgeted, representable endpoint, containment-limited segment, and route-continuing
endpoint before pairwise separation is evaluated. A fixed local selection order favours
route progress while retaining a stable agent-id tie-break; agents are committed only
after all choices are made. This is intentionally a local policy, not a global crowd
solver.

Motion calculations use `double` intermediates. Float endpoint rounding is accepted
only when its observed double displacement is within `max_speed * elapsed`; otherwise
coordinates retreat by ULPs toward the start, with a no-op as the safe fallback. The
stored displacement—not an unsafe float reciprocal—defines reported velocity. Normal
steps are 20 ms; quiescent simulations return early and an extreme duration is consumed
by one final bounded step after a finite fine-step budget.
