# VWmini Repair Pass

Continue work in the current VWmini workspace. Preserve the supplied public API and all
documented requirements. Do not edit the supplied public headers. Diagnose and repair the
following general behaviors; do not hard-code a particular coordinate, mesh shape, agent
ID, path, or trajectory, and do not weaken or remove existing tests.

The initial implementation has a useful private-module structure. Preserve that clarity:
make focused corrections in the geometry, mesh, path, and simulation layers rather than
folding more unrelated policy into one large function.

## 1. Make funnel paths continuously contained

A path around a reflex corner can currently return only `[start, goal]` even though that
segment passes through non-walkable space. The funnel is missing correct terminal-goal
handling and/or uses inconsistent portal orientation.

Implement a standard deterministic portal-funnel/string-pulling path:

- construct consistently oriented portals for the selected cell corridor;
- include appropriate degenerate start and goal portals or otherwise process the final
  goal through the funnel cone correctly;
- return exact caller-supplied endpoints (or one point for exact equality);
- ensure every returned segment is contained under the documented mesh policy; and
- retain deterministic results and `NoPath` for disconnected in-mesh endpoints.

Do not rely only on a same-cell direct-path shortcut. Add tests with a genuinely bent,
concave/reflex corridor where the direct segment exits the mesh, checking every returned
segment and that the route reaches the valid goal. Also retain a direct cross-cell route
when that segment is actually contained.

## 2. Correct mesh topology policy

A complete shared edge uses epsilon-aware matching, but a vertex-only contact is not an
invalid mesh and must create no adjacency. Remove the non-contractual connected-fan or
non-manifold-vertex rejection that rejects otherwise valid vertex-touching triangles.
Keep the required checks for overlapping interiors, non-manifold *edges*, and a vertex
lying in another edge's open interior (T-junction).

Add focused tests for valid vertex-only contact with no route through that contact, and
for the required T-junction/non-manifold-edge rejection cases.

## 3. Make public mutations transactional and IDs durable

`add_agent` currently appends a live agent before validating an optional goal. If that
goal is outside the mesh, it returns an error but leaves the newly created agent present.
Validate everything before mutation, or roll back every mutation on every failing path.
After any failed add, `agent_count`, existing agents, and subsequent IDs must remain
consistent.

Retain the lifetime rule that a removed `AgentId` remains invalid for the lifetime of the
simulation. Add regressions for rejected optional goals, failed-add state preservation,
and stale-ID operations after a remove plus later successful add.

## 4. Repair safe, progressing local avoidance

The current normal-only relative-velocity adjustment can cancel goal-directed motion, so
ordinary crossing and overtaking agents remain `Moving` rather than reaching their valid
goals. It also permits close-following overlap in a feasible reflex-corner route and
does nothing for exactly coincident centres.

Keep decisions deterministic and snapshot-based, but select mutually compatible local
velocities for the *actual simultaneous result*. A small bounded set of deterministic
candidates (desired velocity, slowing/yielding, and tangential sidesteps) with predicted
separation/containment checks is acceptable. For coincident centres use a stable,
deterministic tie-break direction and attempt separation. Do not add global planning,
mutable global state, test-specific priorities, or an unnecessary framework.

In feasible open-space crossing and overtaking cases, agents must stay separated within
the documented tolerance, remain contained and speed-bounded, make progress, and reach
their distinct goals. In close following through a feasible reflex corner, they must not
overlap and must make progress. Add deterministic per-step regressions for all of these,
including initially overlapping recovery.

## 5. Large-duration safety and implementation hygiene

Every finite non-negative `step(float)` duration is valid. Do not cast an unrepresentable
`ceil(seconds / substep)` value to `size_t` or require an impractically unbounded loop
for `FLT_MAX`. Use a safe bounded/chunked approach that preserves finite state,
containment, maximum-speed bounds, and ordinary-duration behavior. Add a focused large
finite-duration regression.

Remove the unused `tri_edge` helper so Clang builds warning-clean. Correct the
`.clang-format` configuration to a supported setting for the available formatter, then
format the project. Keep test builds practical: library-only default configuration should
not unnecessarily require GoogleTest merely to configure a consumer build.

## Documentation and verification

Update `docs/architecture/architecture.md` and
`docs/architecture/implementation-plan.md` to reflect the delivered design, repair
choices, verification, and explicit completion status for every slice. Create or update
`docs/completion-report.md` only with claims supported by the actual submitted code and
tests.

Retain existing tests and add the focused regressions above. Before declaring completion,
build from scratch with GCC and Clang under `-Wall -Wextra -Wpedantic`, run CTest, run an
ASan/UBSan build if available, and run clang-format in check mode. Report only checks
actually run and their results.
