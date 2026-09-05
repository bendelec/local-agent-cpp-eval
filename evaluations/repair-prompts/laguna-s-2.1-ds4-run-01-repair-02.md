# VWmini Final Repair: Restore Contract Foundations

Continue in the current VWmini workspace. This is the final repair pass. Make a small,
cohesive correction of the existing implementation; do **not** start another broad rewrite,
replace files wholesale, weaken/remove tests, or hard-code a coordinate, mesh, agent ID,
route, or trajectory.

The prior repair's native tests pass, but independent public-API conformance is only
**63/77**. The failures are foundational and must be fixed in dependency order below.

## 1. Restore all supplied public headers exactly — first

Restore every file under `include/vwmini/` byte-for-byte to the supplied task baseline before
making implementation changes. In particular, `nav_mesh.hpp` was modified: include ordering,
comments, indentation, access labels, friend/member ordering, and the `m_impl{}` member
initializer differ from the supplied header. Restore the original header exactly.

Do not expose mesh storage, triangles, spans, implementation helpers, or new public API. Keep
`NavMesh::Impl` and all sharing between mesh and pathfinding in private `src/` detail code.
Do not run a formatter over public headers. Before completion, compare the headers byte-for-byte
with the supplied baseline.

## 2. Repair mesh and path predicates before touching crowd motion

Keep the existing private mesh/path structure where useful, but correct these specific contract
errors and add small regressions for each:

- In complete-edge matching, normal non-zero edges are currently skipped because the
  near-endpoint check is inverted. Skip only a zero-length edge; link complete edges whose
  corresponding endpoints are within `epsilon`. This must restore adjacency for ordinary
  squares, L-shaped corridors, and irregular connected meshes while retaining vertex-only
  non-adjacency and rejection of overlaps, T-junctions, and non-manifold edges.
- A strictly interior point of an accepted CCW triangle must be recognized using a strict
  double-precision orientation sign. Do **not** use the distance epsilon as an orientation or
  area threshold. Keep epsilon only for the documented distance-to-a-closed-edge boundary rule.
  A small valid triangle whose double signed area exceeds `epsilon * epsilon` must contain its
  centroid/interior point.
- Use scale-safe intermediates for accepted finite coordinates and edge distances.
- Validate both path endpoints first. Only then return `[start]` when `start == goal` under exact
  `Vec2::operator==`; distinct endpoints within epsilon require a normal path, normally exactly
  `[start, goal]` when visible.
- A directly visible multi-cell segment must return exactly `[start, goal]`. A connected bent or
  irregular corridor must not return `NoPath` merely because its endpoints start in different
  triangles. Every returned segment must be continuously contained under MSH-006. Do not merge
  a real uncovered interval by treating `epsilon` as a parameter-space gap; boundary tolerance
  is a Euclidean edge-distance rule, not a route-length-relative allowance.

Use deterministic, general path logic. Do not solve this by recognizing the test meshes.

## 3. Make lifecycle validation and transitions transactional

Centralize validation used by both `add_agent` and `set_goal`, before mutating observable state.

- Position, goal, radius, max speed, and arrival radius must be finite. Radius and max speed are
  strictly positive.
- `-1.0f` alone means use the agent radius. Every other negative arrival radius is invalid.
  `-0.0f`, zero, and positive finite values are valid explicit values.
- A rejected `add_agent` or `set_goal` must preserve all observable state, including the next
  assignable ID. Do not consume an ID and then erase an unsuccessfully added agent.
- A valid goal already within its effective arrival radius immediately becomes `Reached` with
  zero velocity. A disconnected in-mesh goal becomes `NoPath` with zero velocity.
- Reaching retains the goal value until `clear_goal`; `clear_goal` alone clears it and sets Idle.
  Never snap or teleport an agent to its goal.
- Moved-from simulations must not dereference a null pImpl; queries return their documented
  empty result and mutations return an ordinary error.

Add focused tests for invalid/non-finite scalars, sentinel/zero/negative-zero radii, rollback and
ID continuity, immediate reach, no-path, clear/remove, and final-waypoint exhaustion with zero
arrival radius.

## 4. Correct bounded simultaneous stepping and ordinary crowd progress

Do not use direct positional pushes as an avoidance mechanism. They bypass the speed budget,
are sequential rather than snapshot-based, and can move agents outside the mesh.

For each internal substep, derive desired velocities from an immutable snapshot, choose bounded
deterministic local velocities, then commit all positions together. A small candidate set is
enough: desired, reduced/yielding, stop, and stable tangential/separating candidates. Check
predicted disc separation, speed limits, and containment before selection; retain the feasible
choice with the best goal/waypoint progress and a stable tie-break. Coincident centres need a
stable ID-derived escape direction. Do not allow endpoint-only projection to hide a swept path
through non-walkable space.

The current closing-rate calculation subtracts the combined radius twice: its `gap` already is
`distance - combined_radius`, so a one-substep non-overlap bound must be derived from that gap,
not from `gap - combined_radius`. Correct this as part of a candidate-based simultaneous scheme,
not as an isolated fixture patch.

Feasible ordinary crossing, overtaking, and close reflex-corner following agents must remain
finite, contained, speed-bounded, and separated within tolerance, and reach their distinct goals.
Initially overlapping/coincident agents require a deterministic finite contained separation
attempt. No global planner or dense-crowd guarantee is required.

`step` must also terminate for every finite non-negative duration. Repeatedly subtracting `0.05f`
from `FLT_MAX` stagnates and never terminates. Use a numerically safe duration/chunk strategy;
when agents become terminal/quiescent, consume residual elapsed time without simulating billions
of no-op substeps. Never use unsafe float-to-integer conversion, discard accepted elapsed time,
teleport, or exceed speed bounds.

## 5. Honest tests, documentation, and verification

Keep all existing tests and add concise contract regressions for the cases above. Native tests
must use valid triangle lists for `NavMesh::create`; do not hide a failure by allowing forbidden
goal snapping or by testing only one simplified scene.

Update architecture/plan text only after code is correct. Remove claims that name missing source
files or describe snapshot/candidate avoidance unless that is actually implemented. Do not claim
formatting, sanitizer, compiler, or test results that were not run.

Before completion:

1. Configure and test from a clean build with GCC and `-Wall -Wextra -Wpedantic`.
2. Configure and test with Clang under the same warnings.
3. Run ASan/UBSan if available.
4. Run clang-format check mode on the whole submitted tree, excluding generated build trees.
5. Verify a library-only configure with `BUILD_TESTING=OFF`.
6. Report exact commands and actual results only.

The target is the documented contract, not merely the old native suite. Preserve the supplied
API exactly throughout.
