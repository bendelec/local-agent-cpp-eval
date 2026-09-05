# Repair Follow-up

Continue in the current VWmini workspace. Preserve the supplied public API, documented
requirements, and all working behavior already present. Do not replace files wholesale,
weaken tests, or hard-code a coordinate, mesh, agent id, route, or fixture-specific
trajectory.

Work at a deliberate pace. There is no request for a shortcut or arbitrary deadline, and
scope discipline does not mean avoiding a necessary local refactor. Prefer a small,
well-understood set of cohesive corrections and focused regressions over a quick
superficial patch. Do not add frameworks, third-party dependencies, global state, or
unrequested capabilities.

The current smoke test passes, but the implementation has the concrete contract defects
below. Address their underlying policies, not only the examples.

## 1. Make mesh predicates, complete-edge matching, and path containment sound

Use scale-safe intermediate arithmetic for accepted finite coordinates. A valid large
finite triangle must recognize its strictly interior point; float overflow in orientation,
dot-product, or distance calculations must not turn finite geometry into a false
`contains` result.

Derive adjacency from complete, non-zero-length shared edges whose corresponding endpoints
are within the documented epsilon. Do not require exact `Vec2` equality. Preserve
rejection of true overlaps, non-manifold edges, and T-junctions, while accepting the
small endpoint differences that the contract explicitly permits.

Replace the fixed sample-count route check with a continuous segment-in-union-of-triangles
predicate. Every point of every returned segment must be contained under the stated
boundary rule. In particular, a contained direct route must return exactly `[start, goal]`
with the caller's exact endpoints, including endpoints admitted by boundary tolerance.
Keep deterministic connected routing and meaningful corridor shortening; do not solve this
by accepting routes through non-walkable space or by recognizing test geometry.

Add deterministic native regressions for finite extreme-coordinate containment,
epsilon-matched complete-edge connectivity, boundary-tolerance direct paths with exact
endpoints, and continuous containment of an irregular connected route.

## 2. Make every valid finite step duration terminate safely

`Simulation::step` accepts every finite non-negative duration. Repeatedly subtracting a
fixed small float chunk from a very large finite duration can stagnate and never return.
Use a numerically safe chunking/integration strategy that terminates without unsafe
float-to-integer conversion, rejected valid input, teleportation, discarded elapsed time,
or speed-limit violations. When agents become terminal or quiescent, consume any residual
elapsed time without simulating an impractical number of no-op iterations.

Keep invalid-duration calls transactional, preserve ordinary-duration behavior, and retain
finite contained state. Add a focused regression using
`std::numeric_limits<float>::max()` with a valid moving agent; it must return, remain
finite and contained, obey its speed bound, and reach or correctly retain its valid state
without an unbounded loop.

## 3. Replace repulsion-only avoidance with bounded simultaneous choices

The current repulsion heuristic allows substantial overlap during feasible crossing and
overtaking. At each internal substep, derive decisions from one immutable snapshot of all
live agents. Select bounded candidate velocities that include progress, yielding/stopping,
and stable separating/tangential alternatives, checking predicted disc separation,
containment, and speed limits before committing all chosen positions together.

Normal non-overlapping crossing and overtaking agents in open space must remain separated
within the documented tolerance and reach their distinct goals. Preserve deterministic
behavior, waypoint/goal no-overshoot, finite values, and contained positions. Do not
special-case a fixture, disable avoidance, or introduce a global crowd planner. Retain a
deterministic finite separation attempt for initially overlapping/coincident agents.

Add focused deterministic regressions that inspect separation after every returned step
for crossing and overtaking, as well as the large-duration case above.

## 4. Make the delivered engineering evidence truthful and useful

Split the single implementation unit only where that makes geometry policy, mesh/path
policy, and mutable simulation policy independently understandable and testable. Keep the
result direct; a framework or broad redesign is not wanted.

Replace the debug-only `assert` smoke test with deterministic tests whose checks remain
active in Release builds. Cover successful and rejected paths for each repaired policy.
Honor `BUILD_TESTING=OFF` for a library-only configuration. Format the submitted source
with the available clang-format and keep GCC and Clang warning-clean under
`-Wall -Wextra -Wpedantic`.

Update the architecture document and implementation plan only after the code and tests
are correct. They must describe the actual storage, routing, stepping, tests, and
verification that you deliver. Before completion, run the native Debug and Release test
builds, a GCC and Clang warning-clean build, an ASan/UBSan build if available, and a
library-only `BUILD_TESTING=OFF` configure. Report only commands that were actually run
and their actual outcomes.
