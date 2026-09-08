# VWmini Architecture

This document describes the design of **this** implementation of the VWmini contract
(`docs/requirements/*`, `docs/architecture/public-api.md`). It is the design that
`docs/architecture/implementation-plan.md` planned and that `src/` realises; it does not
prescribe or imitate any external architecture. (NFR-009)

## 1. Forces from the contract

| Constraint | Consequence for the design |
|---|---|
| NFR-002 fixed public surface | Public headers stay untouched; everything else sits in `src/` behind `NavMesh::Impl` / `Simulation::Impl`. |
| NFR-003 no globals, no output, no third-party | Pure functions over explicit inputs; failures travel as `Result<T>`. |
| MSH-007 immutable, transactional mesh | Construction validates into a separate value and only then builds the shared immutable `Impl`. |
| MSH-006 one containment policy | One function decides "is this point walkable"; path and agent code both call it. Acceptance validation applies its own geometric rules (edge spans, strict interiors, crossings) and never asks the walkability question. |
| SIM-002..004 deterministic, contained route | Cell corridor + shortening whose every segment is *verified* contained; deterministic search order and tie-breaks; no floating-point-keyed containers. |
| SIM-010 simultaneous local avoidance | Each substep reads an immutable snapshot of agents and applies updates afterwards. |
| SIM-012 no teleport, never leave the mesh | Integration shrinks a step until its endpoint is contained instead of clamping or teleporting. |

## 2. Components

Small single-responsibility translation units; dependencies point downward only, there is
no inheritance, no dynamic dispatch, and no state outside an owning object.

| Module | Responsibility | Depends on |
|---|---|---|
| `geometry.cpp` | Public `length`, `normalized` (MSH-001). | stdlib |
| `predicates.hpp` | Header-only primitives: signed area, strict point-in-triangle, point/segment distance, segment intersection, `distance`, and the finiteness helpers (`is_finite`, `all_finite`). Owns the `epsilon` policy. | `geometry.hpp` |
| `error.hpp` | `detail::make_error`: the only place a public `Error` value is built, so codes and message text stay consistent. | public headers |
| `triangulation.{hpp,cpp}` | Validate one outline and ear-clip it into CCW triangles (MSH-002/003). | `predicates.hpp` |
| `mesh_topology.{hpp,cpp}` | The validated triangle set plus derived adjacency; every mesh geometry query (`contains`, `locate`, `shared_edge`, `segment_is_contained`) and the whole acceptance validation (MSH-004/005/006). | `predicates.hpp` |
| `nav_mesh_impl.hpp` | Defines `NavMesh::Impl`; its single member is a `MeshTopology`. | `mesh_topology.hpp` |
| `nav_mesh.cpp` | Public façade `NavMesh::create/contains/cell_count`; turns an internal failure into a public `Error`. | `nav_mesh_impl.hpp` |
| `corridor.{hpp,cpp}` | Deterministic Dijkstra over cell adjacency between the cells containing start and goal (SIM-004). | `mesh_topology.hpp` |
| `pathfinding.cpp` | `find_path`: endpoint validation, direct-line shortcut, portal-seeded string pulling, output simplification (SIM-001/002/003). | `corridor`, `mesh_topology`, `nav_mesh_impl.hpp` (friend seam to read `NavMesh::Impl`) |
| `agent.hpp` | `AgentRuntime`: one agent's mutable state plus the transitions that depend only on that state (goal adoption, goal clearing, target selection, arrival). | `simulation.hpp`, `predicates.hpp` |
| `steering.{hpp,cpp}` | Local avoidance: one agent's velocity for one substep, chosen from a snapshot of all agents (SIM-010/011/012). | `agent.hpp`, `predicates.hpp` |
| `simulation.cpp` | `Simulation::Impl`: agent table and id allocation, validation of every public mutation, route assignment on goal set (`route_to_goal`: `Moving` / `Reached` / the non-error `NoPath`), substepped advancement with snapshot decisions. | `agent.hpp`, `steering.hpp`, public `find_path`, `error.hpp` |

`Simulation` talks to the mesh only through the public `NavMesh` API
(`contains`, `find_path`); it does not reach into `MeshTopology`. That keeps the crowd code
independent of how the mesh is stored.

## 3. Dependency graph and ownership

Arrows point at the dependency (`A --> B` means A uses B). Everything below the public
headers is internal.

```
nav_mesh.cpp    --> nav_mesh_impl.hpp --> mesh_topology --> predicates.hpp --> geometry.hpp
pathfinding.cpp --> corridor          --> mesh_topology          (friend of NavMesh::Impl)
triangulation.cpp --> predicates.hpp
simulation.cpp  --> steering --> agent.hpp --> predicates.hpp
simulation.cpp  --> error.hpp --> (public Error)
simulation.cpp  --> public NavMesh::contains / find_path only     (never MeshTopology)
```

* No cycles: `predicates.hpp` is a leaf, `mesh_topology` knows nothing about
  `corridor`/`pathfinding`, and the crowd layer cannot see mesh internals.
* `NavMesh::Impl` (`nav_mesh_impl.hpp`) and `Simulation::Impl` are the only seams between the
  fixed public surface and the implementation; `pathfinding.cpp` is a friend of `NavMesh::Impl`
  so `find_path` can read the topology without exposing it in a public header.
* Ownership: `NavMesh` holds one `shared_ptr<const Impl>` (immutable, copyable, allocation-free
  copies); `Simulation` owns its `Impl` exclusively and inside it a `std::vector<AgentRuntime>`
  with free-slot recycling. No raw owning pointers anywhere.
* Data flow: `vector<Polygon>` in → validated `MeshTopology` → immutable `Impl`; `Vec2` endpoints
  in → `Path` out; `AgentConfig` in → `AgentId` + table entry → per-substep
  snapshot → velocities → positions.

## 4. Data flow

**Mesh creation**: `NavMesh::create` → `MeshTopology::build` (copies the input, validates
finiteness → `InvalidArgument`, geometry → `InvalidMesh`, derives adjacency) → on success
wrap in `Impl`; on failure nothing is exposed (MSH-007).

**Containment** (`MeshTopology::contains`): walkable when strictly inside a triangle, or
when the distance to a triangle's closed edge segment is ≤ `epsilon` — MSH-006 exactly.
Const, allocation-free, no caching.

**Path query**: `find_path` → validate endpoints (`InvalidArgument` / `OutsideMesh`) →
exactly-equal endpoints → `[start]` → straight line contained → `[start, goal]` → else
`corridor` (Dijkstra, cost = centroid distance, tie-break by cell index) → seed a polyline with
both endpoints of every portal the consecutive corridor cells share (`MeshTopology::shared_edge`;
consecutive portal endpoints lie in one convex cell, so the seed is contained by construction) →
pull the string by jumping to the furthest waypoint whose connecting segment is verified
contained → drop intermediate points closer than `epsilon`, keeping the caller's own start and
goal, with a final containment check that reverts an unsafe drop (SIM-002/003).

**Goal assignment** (`Simulation::Impl::route_to_goal`, used by `add_agent` and `set_goal`):
query `find_path` for the new goal. A connected goal farther than the effective arrival radius
adopts the route and becomes `Moving`; a goal already inside that radius clears the route, zeroes
velocity and becomes `Reached`; a disconnected in-mesh goal is *not* an error — it clears the
route, zeroes velocity and becomes `NoPath` (SIM-007). Validation happens before any field of the
agent is touched, so a rejected call leaves the previous goal and route intact.

**Stepping**: `step(seconds)` → validate (`InvalidArgument` on non-finite/negative, no
state touched) → `1 … kMaxSubsteps` substeps. Each substep:
1. copy the agent table into a snapshot;
2. per agent: desired velocity toward its current target (`AgentRuntime::target`);
3. `steering::avoid_velocity` adjusts it against the snapshot;
4. `advance_agent` integrates it: stride capped by the distance to the target, then shrunk until
   the endpoint is contained, then waypoint capture and `refresh_arrival`.

## 5. Error handling

Modules that can reject caller input report `Result<T>`/`Error` themselves
(`triangulate_outline`, `MeshTopology::build`); `pathfinding.cpp` and `simulation.cpp` return the
public `Result<T>` directly, while helpers signalling a local failure use `std::optional`/`bool`.
`error.hpp` is the single place that
constructs them. Validation paths are separate from the stepping path, so a rejected call
returns before any state is touched (SIM-008, MSH-007).

## 6. Key decisions and trade-offs

1. **Triangles stay triangles.** Accepted input is stored cell per cell — no vertex welding,
   no half-edge structure. Adjacency is derived once at build time by pairwise edge matching
   within `epsilon`, which is what MSH-005 literally describes. Cost: O(n²) build, O(n)
   containment. Gain: the validation rules (non-manifold, T-junction, interior overlap) read
   directly off the submitted geometry; NFR-008 sets no performance target.
2. **Validation before mutation.** `MeshTopology::build` works on a copy and returns a fresh
   value, so a rejected submission cannot leave a partial mesh behind.
3. **Segment containment by critical-parameter sampling.** For a union of convex triangles
   the covered part of a segment is a union of intervals whose endpoints are edge crossings,
   so testing one point per interval between consecutive critical parameters decides
   containment. Disagreements below `epsilon` fall in the numerically ambiguous band the
   requirements place outside the interoperability guarantee.
4. **Verified greedy shortening instead of a hand-written funnel.** The corridor is shortened
   by repeatedly jumping to the furthest corridor waypoint that is reachable by a segment
   `MeshTopology::segment_is_contained` accepts. Every retained segment is therefore
   contained *by construction and by verification*, SIM-003's "funnel-equivalent" is
   satisfied (the result is taut: skipping a waypoint leaves an unverifiable segment), and
   ~120 lines of portal bookkeeping disappear. Trade-off: routes are taut but not proven
   optimal, which SIM-004/MSH-008 explicitly leave open.
5. **Collision-cone deflection instead of an ORCA solver.** For each peer inside a fixed
   look-ahead horizon the agent rotates its relative velocity onto the nearer edge of that
   pair's collision cone, keeping speed: the smallest rotation that makes the two discs miss.
   No tuned repulsion weights, no linear program, no candidate sampling — which makes the
   choice easy to explain and trivially deterministic (SIM-012). Trade-off: it is greedy and
   pairwise, so it makes no progress guarantees in layouts that need global planning
   (explicitly out of scope, SIM-011/SIM-012).
6. **Overlapping discs separate; nobody avoids "backwards".** Pairs that already overlap
   (allowed by SIM-010's edge case) drive apart along the centre line, with a fixed axis for
   exactly-concentric discs so the response stays deterministic. Peers that are separating,
   or that only come close beyond the look-ahead horizon, are ignored, so a follower does not
   make a leader brake.
7. **Containment-safe integration rather than post-hoc repair.** A stride is capped by the
   distance to the target and, if its endpoint leaves the mesh, halved repeatedly (bounded
   iterations) before the agent moves at all. Motion therefore stays continuous and inside
   the mesh without teleporting (SIM-008/SIM-012); the price is that an agent pressed against
   a wall can stall for a substep.
8. **Every tolerance is a named constant.** The contract `epsilon` (`predicates.hpp`) is used
   where a requirement names it (point/edge distance, shared-edge matching, waypoint omission)
   and, deliberately, also for the two route-state tolerances that measure the same geometric
   scale: dropping the already-occupied route head (`agent.hpp`) and `kWaypointTolerance`
   (`simulation.cpp`). Every other tolerance is its own constant with a stated meaning
   (`kLookAhead`, `kPreferredSubstep`, `kMaxSubsteps`, `kContainmentShrink`,
   `kContainmentAttempts`, test tolerances), so unrelated rules never silently share a value.

## 7. Determinism rules

* Fixed iteration order everywhere: cells by index, agents by insertion order; no hash
  containers, no pointers as ordering keys.
* Dijkstra ties break on cell index; steering peers are visited in table order and the
  deflection side is chosen by the sign of one deterministic angle.
* Substep count is `min(seconds / kPreferredSubstep + 1, kMaxSubsteps)`, clamped in the float
  domain before the `int` conversion — a pure function of the argument, so the same call always
  performs the same arithmetic.
* Nothing reads clocks, randomness, locale or environment; identical inputs on one platform
  give bit-identical outputs (SIM-004).
