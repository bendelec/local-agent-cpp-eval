# Repair Follow-up

Continue improving the current VWmini library. Preserve the supplied public API,
requirements, and the behavior that already works. Make a focused repair; do **not** replace the
project wholesale, weaken/delete tests, hard-code a coordinate, mesh, agent id, route, or
scenario-specific trajectory, or add third-party dependencies, global mutable state, or a global
crowd planner.

A review of the completed library found the valid-input issues below. Address the underlying
policies generally, then add concise deterministic regression tests through the project's normal
CTest setup.

## 1. Make every finite positive substep numerically safe and speed-bounded

`Simulation::step(std::numeric_limits<float>::denorm_min())` can leave a moving agent with a NaN
velocity. The current integration computes `displacement * (1.0F / seconds)`; the reciprocal of a
subnormal duration overflows before the multiplication. This violates the finite and speed-bounded
motion requirements.

Compute observed displacement and velocity without an unsafe float reciprocal. Use widened
intermediates or make an unrepresentable/no-op stored movement explicitly report zero velocity.
For all finite positive durations, positions and velocities must remain finite and the observed
velocity must not exceed `max_speed` (within ordinary floating tolerance).

Storage rounding also needs an explicit policy. On a broad valid mesh, start `{10000,0}`, speed
`1`, and duration `0.0006f`, the current code stores a displacement of `0.0009765625`, greater
than the `0.0006` speed allowance. Choose a representable stored endpoint whose actual
double-measured displacement is no greater than `max_speed * elapsed`; a no-op is preferable to a
one-ULP overspeed. Base the reported velocity on that stored displacement safely. Do not
accumulate hidden residual motion that later teleports or exceeds a future step budget.

Also make the duration-subdivision loop terminate for every finite non-negative `float`.
Repeated subtraction from a very large float can stagnate. Do not simulate billions of no-op
substeps; preserve normal-step behavior and consume remaining time safely when agents are
quiescent.

Add deterministic regressions for the denormal-duration and half-to-one-ULP far-origin cases,
including finiteness, containment, observed speed, and unchanged state where no representable
legal movement exists.

## 2. Repair local avoidance so feasible agents remain separated and make progress

The snapshot-based intent is sound, but the final collision correction followed by individual mesh
clipping does not guarantee that actually committed endpoints remain separated. A close pair
rounding an inside reflex corner can reach about `0.49849` separation when the allowed minimum is
`0.499`, after which the trailing agent stalls near the corner.

A normal open-space crossing also fails to complete: after a generous time budget both agents can
remain `Moving`, near `x=7.979`, repeatedly constrained at or near exactly their combined `0.5`
radius separation rather than entering their distinct arrival radii.

For every internal substep, base all decisions on one immutable position/velocity snapshot.
Evaluate the **actual feasible stored motions**—after containment limitation and float storage
rounding—when enforcing pair separation. Select deterministic local velocity options that retain a
valid route/progress option (desired, yield/reduced, and stable tangential/separating choices are
enough); do not make sequential updates or positional pushes. A choice that preserves separation
only by indefinitely stopping a feasible agent is not adequate. Keep normal agents finite,
continuously contained, actually speed-bounded, and separated within the stated tolerance. Agents
in small feasible crossing, overtaking, and close corner-following situations must reach their
distinct goals. Initially overlapping agents still need a deterministic finite contained
separation attempt.

No named avoidance algorithm, global planner, or arbitrary-density guarantee is required. Do not
relax the separation tolerance or snap/teleport an agent to its goal.

Add focused deterministic tests for crossing completion, corner-following completion, and the
tight post-step separation check. Keep the existing tests; assertions must check public
state/invariants rather than a private exact trajectory.

## 3. Keep the repair understandable

The Geometry → immutable Mesh/Path → mutable Simulation dependency flow and pImpl ownership are
appropriate. Preserve that. `NavMesh::create` currently concentrates validation, topology, and
component construction in one long routine, while the simulation helper mixes steering, predicted
overlap correction, containment bisection, integration, and route-status updates. Extract only
cohesive private helpers that make the numeric and avoidance invariants independently
understandable and testable; do not introduce framework-like abstractions or duplicate geometry
rules. In particular, name the difference between intended velocity, mesh-feasible displacement,
stored endpoint, and committed velocity.

Update the architecture and implementation-plan documents after code and tests pass so they
describe the actual stepping/avoidance boundary and this repair. Do not claim a verification result
that was not run.

## 4. Strengthen project quality evidence

The current small custom assertion executable misses the numeric and crowd cases above. Retain it
or replace it only with an equally clear CTest-integrated setup, and extend coverage across the
repaired success and failure paths. Tests must remain fast and deterministic.

NFR-007 requires consistently formatted source. The tree has no checked formatting configuration,
and the available `clang-format --dry-run --Werror` reports the C++ files as unformatted.
Establish and apply one checked project style without changing the fixed public API, then make the
submitted C++ tree pass the corresponding clang-format check.

Before completion, run and report actual outcomes for:

1. clean Debug and Release GCC builds/tests with `-Wall -Wextra -Wpedantic`;
2. a clean Clang build/test with the same warnings;
3. ASan/UBSan tests if available;
4. a `BUILD_TESTING=OFF` library-only configure/build; and
5. clang-format check mode over all submitted C++ sources and headers, excluding generated build
   trees.

Preserve all fixed signatures, defaults, enums, error meanings, include paths, and the
`vwmini::vwmini` target.
