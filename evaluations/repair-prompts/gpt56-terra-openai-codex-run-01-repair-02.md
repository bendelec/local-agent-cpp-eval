# Final Repair Follow-up

Continue improving the current VWmini library. This is a focused final repair: preserve the public
API, the working ordinary-scale geometry/path behavior, the successful lifecycle and stepping
work, and the small direct C++ design. Do **not** replace the project wholesale, remove or weaken
tests, hard-code coordinates, meshes, agent IDs, routes, or scenario-specific behavior, or add
third-party libraries, global mutable state, or a global crowd planner.

The prior repair correctly addressed ordinary stored-position rounding, tiny durations, build
configuration, formatting, and several local-avoidance cases. Review of the repaired library
found the remaining issues below. Fix their shared causes rather than special-casing the examples.

## 1. Use a consistent double-precision geometry policy for accepted finite input

Several mesh operations widen operands too late. In particular, segment containment forms
`Vec2` differences and calls public float `dot`/`cross` before assigning their results to doubles.
For finite extreme coordinates, those float intermediate calculations overflow. A valid L-shaped
outline with vertices proportional to `FLT_MAX` can be triangulated and turned into a valid mesh;
its missing corner does not contain `{0,0}`, yet a path from `{0.6F * FLT_MAX, -0.6F * FLT_MAX}`
to `{-0.6F * FLT_MAX, 0.6F * FLT_MAX}` is incorrectly returned as one direct segment through the
missing corner.

Keep the public `Vec2` API unchanged, but use a private double-coordinate representation for
all geometry decisions: orientation, signed area, vector differences, dot/cross products,
point-to-segment distance, clipping/interval calculations, topology checks, containment, and
path length. Do not use public float vector arithmetic inside these predicates. A finite input
that has been accepted by `NavMesh::create` must not produce an overflow-driven direct route,
false containment result, or non-finite internal decision.

Correct the area unit consistently. `orient(a,b,c)` is **twice** signed triangle area, while the
requirements define the non-degenerate threshold using signed area. A triangle with vertices
`{0,0}`, `{0.00015f,0}`, `{0,0.0001f}` has signed area `7.5e-9`, below `epsilon * epsilon`, and
must be rejected. Apply the correct factor both to literal mesh triangles and to every triangle
emitted by ear clipping; do not accidentally reject valid ordinary small triangles.

Also make `normalized` safe for every finite non-zero vector. A denormal x-axis vector currently
produces `{inf, NaN}` from a float reciprocal, and a maximum-finite diagonal collapses to zero
when its float norm overflows. Use scaled or widened normalization so a finite non-zero input
returns finite, non-zero unit-direction components as representable. Keep zero normalization
exactly `{0,0}`. Be explicit and consistent about any unavoidable `float`-return saturation
policy for `length`; never let that policy corrupt predicates or normalization.

Add deterministic tests for all of the above: the sub-threshold triangle rejection, a nearby
valid triangle acceptance, finite extreme-coordinate containment/path behavior on a non-convex
mesh, denormal normalization, and a maximum-finite diagonal direction. Retain all ordinary-scale
coverage.

## 2. Make mesh-limited local motion retain a usable route through a reflex corner

A close follower in a valid L-shaped passage still becomes permanently `Moving` with zero velocity
at the inside corner while its leader reaches its own goal. The follower can be clamped to a
near-boundary position around `{1.0001,1.0001}`. That position is not usable by the same routing
query for its next portal/goal, so later substeps have no route-feasible movement even though the
original route is feasible and there is ample separation from the stopped leader.

A mesh safety clamp must not strand an agent on an endpoint from which its retained or refreshed
route is unusable. Evaluate the actual representable endpoint with the same complete-segment and
containment policy used by routing. If an endpoint is only a tolerance-fringe artifact or cannot
continue the route, back off to a valid earlier endpoint or refresh a valid route before the next
motion decision. Preserve exact route endpoints when a normal path query returns them; do not
teleport, push an agent through the corner, relax separation, or convert a reachable goal to
`NoPath` merely because local steering was clipped.

Keep decisions snapshot-based and deterministic. Pair checks must continue to use the exact
motions that will be committed after mesh limiting and float storage. Non-moving agents must not
be assigned a hypothetical avoidance displacement that is later discarded during commit; their
only planned motion is stationary. This avoids approving a pairwise choice different from the
actual state update.

Add a concise literal-triangle L-passage test with close followers and distinct terminal goals.
At every step it must check finite state, mesh containment, actual speed, and disc separation;
both agents must reach. Include a regression that exercises the former near-corner clamp rather
than only a wider or differently triangulated passage.

## 3. Keep the final repair small, understandable, and honest

The updated simulation has useful named concepts (`PlannedMotion`, intended velocity, stored
endpoint), but the selection routine now combines nested pair iteration, option search,
compatibility, priority scoring, and mutation. Keep the final correction local. If a helper is
needed, give it one invariant-focused responsibility (for example, construct a committed
route-continuing motion, or select a compatible pair), rather than adding another policy layer.
Do not duplicate mesh geometry in simulation: use one private, scale-safe mesh query seam.

Update the architecture and implementation-plan documents after the code works. They must not
claim that component membership or a mesh clamp prevents vertex/boundary route problems unless
the submitted implementation actually guarantees it. Keep test descriptions and verification
claims factual.

Before completion, run and report actual outcomes for clean GCC Debug and Release builds/tests
with `-Wall -Wextra -Wpedantic`, a Clang build/test with the same warnings, ASan/UBSan when
available, a `BUILD_TESTING=OFF` library-only configure/build, and the checked clang-format
target or equivalent full-tree clang-format check. Preserve every fixed signature, default, enum,
error meaning, include path, and the `vwmini::vwmini` target.
