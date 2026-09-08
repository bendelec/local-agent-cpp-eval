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
  with monotonic ids and erase-on-remove. No raw owning pointers anywhere.
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
4. `advance_agent` integrates it: stride capped by the distance to the target, shortened until the
   mesh covers the whole segment to the landing position, then waypoint capture and
   `refresh_arrival`.

## 5. Error handling

Modules that can reject caller input report `Result<T>`/`Error` themselves
(`triangulate_outline`, `MeshTopology::build`); `pathfinding.cpp` and `simulation.cpp` return the
public `Result<T>` directly, while helpers signalling a local failure use `std::optional`/`bool`.
`error.hpp` is the single place that
constructs them. Validation paths are separate from the stepping path, so a rejected call
returns before any state is touched (SIM-008, MSH-007).

## 6. Numeric policy

`src/predicates.hpp` is the single place that decides how geometry is computed; `src/vec2d.hpp`
supplies the double working vector `Vec2d`, which is implicitly constructed from a `Vec2`
(float to double is an exact conversion).

* **Every geometric decision is taken on double intermediates**: orientation and signed area
  (`signed_double_area`), containment (`strictly_inside_triangle`, `cell_contains`,
  `MeshTopology::segment_is_contained` and its critical-parameter samples), point/segment
  distance and projection, segment intersection parameters, segment crossing, ring area
  (`ring_double_area`), corridor step costs and cell centroids, waypoint dropping, and the
  relative-motion and collision decisions in steering and motion. A product of two floats is
  exact in double and double covers a far wider exponent range, so signs and comparisons stay
  meaningful for every accepted finite float input. Measured on the triangle
  `(0,0) (3e38,0) (0,3e38)`: the edge cross product is `inf` in float but `3e76` in double, and
  `2.5e38 + 1.5e38` is `inf` in float but `4e38` in double - a decision taken in float there is
  made from an overflowed value.
* **Operands are widened one at a time.** Subtracting or multiplying two `Vec2` values performs
  that arithmetic in `float`, so internal code converts each operand to `Vec2d` before computing
  with it. `Vec2`'s own operators stay `float` because they are contract types; they are not used
  to reach a decision. A review round found four sites that computed a difference in `float` and
  widened afterwards - a segment whose span exceeds `FLT_MAX` produced an `inf` delta and a
  spurious route detour - so the rule is written down instead of left to habit.
* **No decision may come from an overflowed, infinite or NaN intermediate.** Predicates report
  through plain `bool` sign tests and through appended parameters (`append_crossing_parameters`
  adds only the crossings that exist to the caller's vector) rather than returning a value that a
  caller might compare against without knowing it came from a non-finite intermediate.
* **`epsilon` stays where the contract puts it.** It is used for point/edge distance and
  complete-edge endpoint matching (`kEpsilon`, with `kEpsilonLength` / `kEpsilonSquared` for the
  double comparisons), and - deliberately, see §7.8 - for the two route-state rules that measure
  the same metre scale: dropping a waypoint the agent already occupies and dropping a waypoint it
  has reached. There is no orientation or area tolerance: a triangle is non-degenerate
  exactly when |signed double area| > epsilon * epsilon (`mesh.md`).
  Epsilon comparisons keep the resolution of the stored coordinates, which is a real limit: beyond
  roughly 1e3 m from the origin one float ULP exceeds `epsilon`, so an epsilon test degenerates to
  exact equality there. Containment, incidence and routing stay usable at any accepted finite
  coordinate; an epsilon tolerance does not, so a tolerance that must work at that scale is stated
  as a fraction of the object's own size.
* **Stored coordinates are never rewritten.** `Vec2` remains the storage and public type;
  widening happens per operation. Values that must be stored back as floats pass through one
  narrowing point (`Vec2d::narrow()`, and the `float` returns in `geometry.cpp`). `narrow` checks
  finiteness and the `[-FLT_MAX, FLT_MAX]` range *before* converting and answers `nullopt`
  otherwise, so an unrepresentable value never becomes an implementation-defined `float` (`NaN`
  fails the same comparison, so it is rejected by the same test).
* **Public norms.** `length` is `hypot` evaluated in double: when the true norm exceeds the
  largest finite float the result saturates at `FLT_MAX`, so a finite vector never produces
  `inf`; a non-finite input propagates. `normalized` first divides by the largest-magnitude
  component, so an extreme or denormal vector still yields a unit direction; only a zero or
  non-finite input yields `{0, 0}`.
* **A displacement the stored position cannot hold does not happen.** A substep's displacement
  is computed in double and converted through the checked `narrow()`: when the landing point is
  not a representable `Vec2`, the candidate is dropped and the agent stays in place for that
  substep (the shortening rule of SIM-012). Nothing is carried between substeps, so
  `distance <= max_speed * elapsed` holds for every individual call and for every cumulative
  interval, at any coordinate. An agent so far from the origin that one substep's stride falls
  below one float ULP of its own coordinate simply keeps its position (at 2e38 m one ULP is about
  2e31 m) — a documented consequence of storing positions as `float`, not a remainder to bank and
  repay.
* **A substep's whole path is checked, not only its endpoint.** A candidate displacement is
  accepted only when `MeshTopology::segment_is_contained` — the exact interval-coverage query that
  certifies a planned route — reports the segment from the current position to the landing
  position completely inside the mesh. Unobstructed motion costs one such query; a stride that
  does not fit is bisected (`kMotionBisectionSteps`, enough to leave the shortened remainder below
  one float ULP of the landing coordinate) until it does, so an agent pressed against a wall
  stalls instead of jumping. Coverage along a ray is not monotone in general, so bisection returns
  a *verified* fraction rather than necessarily the largest one: confirming the candidate, not
  maximising it, is what SIM-012 asks for. Sampling two endpoints and a midpoint was tried first
  and discarded — it accepted a jump across two gaps when start, middle and endpoint happened to
  sit in three different walkable triangles.
* **Motion asks the mesh, not a copy of it.** `Simulation` reaches the validated topology through
  a single private seam — `NavMesh` grants `Simulation` friendship, and no public declaration of
  `NavMesh` changes — so there is exactly one implementation of the walkability question
  (MSH-006). What is deliberately *not* modelled: an agent is a point. Its radius enters steering,
  never the containment test, so the swept disc is never tested against walls.

* **Speed caps hold after narrowing.** The double velocity is limited before it becomes a `Vec2`,
  and because rounding can put the stored magnitude one float ULP over `max_speed`, the narrowed
  value is pulled back under the limit (`steering.cpp`): the cap is a contract, not a target.
* **Steering tolerances are relative.** The "not moving" speed and the "concentric" centre distance
  are fractions of the pair's own speed and radii (`kStillSpeedFraction`, `kConcentricFraction`),
  so avoidance behaves the same at 1 mm/s and at 1e6 m/s.
* **Numeric regressions are Release-active.** They are GTest assertions in
  `tests/numeric_test.cpp`, not `assert()`, so the `NDEBUG` Release build keeps them.

## 7. Key decisions and trade-offs

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
   distance to the target and, if the step would leave the mesh, bisected (bounded iterations)
   until `segment_is_contained` confirms the whole swept segment, before the agent moves at all.
   Motion therefore stays continuous and inside the mesh without teleporting (SIM-008/SIM-012);
   the price is that an agent pressed against a wall can stall for a substep.
8. **Every tolerance is a named constant.** The contract `epsilon` (`predicates.hpp`) is used
   where a requirement names it (point/edge distance, shared-edge matching, waypoint omission)
   and, deliberately, also for the two route-state tolerances that measure the same geometric
   scale: dropping the already-occupied route head (`agent.hpp`) and dropping a waypoint the agent has
   already reached (`simulation.cpp`). Every other tolerance is its own constant with a stated meaning
   (`kLookAhead`, `kPreferredSubstep`, `kMaxSubsteps`, `kContainmentShrink`,
   `kContainmentAttempts`, `kStillSpeedFraction`, `kConcentricFraction`,
   test tolerances), so unrelated rules never silently share a value.

## 8. Determinism rules

* Fixed iteration order everywhere: cells by index, agents by insertion order; no hash
  containers, no pointers as ordering keys.
* Dijkstra ties break on cell index; steering peers are visited in table order and the
  deflection side is chosen by the sign of one deterministic angle.
* Substep count is `min(seconds / kPreferredSubstep + 1, kMaxSubsteps)`, evaluated and clamped in
  double before the single `int` conversion — a pure function of the argument, so the same call
  always performs the same arithmetic.
* Nothing reads clocks, randomness, locale or environment; identical inputs on one platform
  give bit-identical outputs (SIM-004).
