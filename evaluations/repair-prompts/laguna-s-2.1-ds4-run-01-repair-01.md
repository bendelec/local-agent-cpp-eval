# VWmini Repair Pass

Continue work in the current VWmini workspace. Preserve the supplied public API and all
documented requirements. Do not hard-code a coordinate, mesh shape, agent ID, route, or
trajectory; do not weaken or remove existing tests. Make focused, cohesive changes rather
than adding a framework or rewriting working geometry/triangulation without cause.

## 1. Restore public-header opacity and use private implementation sharing

The supplied public headers are fixed. Restore `include/vwmini/nav_mesh.hpp` byte-for-byte:
do not expose a `Triangle` type, `std::span`, storage view, or any other mesh-internal
seam. Existing public declarations, types, defaults, layout, linkage, and documentation
must remain unchanged.

`NavMesh::Impl` may be defined in a private internal header under `src/`, shared only by
`nav_mesh.cpp` and `path_finder.cpp`. `find_path` already has the intended friendship;
use that private seam rather than extending the public surface.

## 2. Correct mesh predicates and connectivity

Centralize private finite, orientation, signed-area, segment-distance, and
endpoint-nearness helpers so navmesh and pathfinding do not implement incompatible rules.
Use double or otherwise scale-safe intermediate arithmetic where needed.

- Use strict orientation/sign for a point strictly inside an accepted CCW triangle; the
  distance epsilon belongs only to the documented closed-edge boundary rule. A small but
  valid accepted triangle must contain its interior.
- Use the specified signed double-area threshold for degenerate triangles.
- Match complete shared-edge endpoints within epsilon, not with exact `Vec2` equality.
  Preserve vertex-only contact as valid but non-adjacent.
- Preserve required overlap, T-junction, and non-manifold-edge rejection.

Add focused regressions for a small valid triangle and its interior, epsilon-matched
shared edges, vertex-only non-adjacency, and the required invalid topology cases.

## 3. Implement actual connected contained pathfinding

After endpoint validation and the exact-equality singleton case, return exactly
`[start, goal]` whenever the full direct segment is contained. Do not mistake different
containing triangle indices for disconnected components: traverse the accepted
complete-edge adjacency graph to determine connectivity and construct a deterministic
contained route around a reflex corner. Return `NoPath` only for genuinely disconnected
in-mesh endpoints.

The containment rule applies to every real point of every returned segment. Replace the
capped uniform sampler with an analytical/event/interval coverage test: intersect a
segment with the union of closed triangles, merge covered parameter intervals under the
documented tolerance, and require complete coverage of `[0,1]`. Do not permit routing
across a vertex-only touch.

Retain exact caller-supplied endpoints, deterministic repeated results, and meaningful
path shortening. Add tests for connected bent corridors in both directions, disconnected
components, exact versus merely epsilon-close endpoints, a direct multi-cell path, and
continuous containment of every returned segment.

## 4. Repair lifecycle validation and transactional state transitions

Centralize validation used by `add_agent` and `set_goal`:

- position, optional goal, radius, max speed, and arrival radius must be finite as
  required;
- radius and max speed must be strictly positive;
- `-1.0f` is the sole arrival-radius sentinel; every other negative value is invalid;
  zero and positive finite radii are explicit values;
- rejected operations must leave all observable simulation state unchanged.

For an in-mesh goal already within the effective arrival radius, transition immediately
to `Reached` with zero velocity. A disconnected in-mesh goal transitions to `NoPath`
with zero velocity. Preserve an arrived agent's goal value until `clear_goal`; only
`clear_goal` makes it idle and clears the goal. Removed IDs remain invalid for the
simulation lifetime. Treat a moved-from simulation defensively rather than dereferencing
a null pImpl.

Add regressions for every invalid scalar category, all arrival-radius cases (including
negative zero), failed-operation rollback, immediate reach, no-path, clear/remove/stale
ID behavior, and moved-from queries/operations if they are supported defensively.

## 5. Replace direct positional pushes with bounded simultaneous avoidance

`step` must validate duration and advance the requested finite non-negative duration
without teleporting. Subdivide as needed for ordinary durations. At each substep:

1. snapshot all live agents before any movement;
2. derive desired waypoint/goal velocity, bounded by max speed and no-overshoot;
3. deterministically select compatible local velocities from a small bounded set
   (goal-directed, yielding/slow, stop, and stable tangential/separating alternatives)
   using predicted disc separation, speed bounds, and mesh containment;
4. use a stable ID-derived tie-break/escape direction for coincident centres; and
5. commit all chosen motion together, retaining contained, finite positions and bounded
   reported velocity.

Never directly reposition agents by more than their speed-budget, never snap a reached
agent to its goal, and never apply separation outside containment checks. In feasible
open space, normal crossing and overtaking agents must remain separated within the
documented tolerance, make progress, and reach their distinct goals. Close agents on a
feasible reflex-corner route must remain contained/separated and make progress. The
implementation need not solve arbitrary dense deadlocks.

Add deterministic tests checking every substep's containment, speed bound, and disc
separation, plus eventual reaching for crossing, overtaking, and feasible reflex following;
retain/add exact-overlap recovery and large-duration no-overshoot coverage.

## 6. Build, tests, docs, and verification

Keep the self-contained test framework if desired, but make its failures and assertion
count visible and create focused independent contract tests. Honor `BUILD_TESTING` so a
library-only CMake configure does not build test targets. Remove the unused test constant.
Use a formatter configuration supported by the available clang-format and make format
check mode pass.

Update architecture and implementation-plan documents only after the code and tests are
true. Remove evaluation/grader-oriented wording and claims that do not match the source.
Before completion, build cleanly with GCC and Clang under `-Wall -Wextra -Wpedantic`, run
CTest, run ASan/UBSan if available, and report only commands actually run and results
actually obtained.
