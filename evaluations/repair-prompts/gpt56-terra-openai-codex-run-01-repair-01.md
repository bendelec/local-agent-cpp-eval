# Repair Follow-up — GPT-5.6 terra, round 1

Continue in the current VWmini workspace. Preserve the supplied public API, requirements,
and the substantial working behavior. Make a focused repair; do **not** replace the project
wholesale, weaken/delete tests, hard-code a coordinate, mesh, agent id, route, or
fixture-specific trajectory, or add third-party dependencies, global mutable state, or a
global crowd planner.

The archived first round builds warning-clean with GCC and Clang, passes its native Debug,
Release, and ASan/UBSan tests, and passes geometry **15/15** and navmesh/path **30/30** public
conformance tests. Its lifecycle track passes **31/33**, and crowd passes **2/4**. Retain those
strengths while fixing the four demonstrated valid-input failures below. The target is the
contract, not only these coordinates.

## 1. Make every finite positive substep numerically safe and speed-bounded

`Simulation::step(std::numeric_limits<float>::denorm_min())` leaves a moving agent with a
NaN velocity. The present integration computes `displacement * (1.0F / seconds)`; the reciprocal
of a subnormal duration overflows before the multiplication. This violates **SIM-008** and
**SIM-012**.

Compute observed displacement/velocity without an unsafe float reciprocal. Use widened
intermediates or make an unrepresentable/no-op stored movement explicitly report zero velocity.
For all finite positive durations, positions and velocities must remain finite and the observed
velocity must not exceed `max_speed` (within ordinary floating tolerance).

At a far but valid coordinate, storing an otherwise bounded float endpoint can round one ULP
farther than the frame budget. With start `{10000,0}`, speed `1`, and duration `0.0006f`, the
first round stores a displacement of `0.0009765625`, exceeding `0.0006`. Choose a representable
stored endpoint whose actual double-measured displacement is no greater than
`max_speed * elapsed`; a no-op is preferable to one-ULP overspeed. Base the reported velocity on
that stored displacement, safely. Do not accumulate hidden residual motion that later teleports
or exceeds a future step budget.

Also make the duration-subdivision loop terminate for every finite non-negative `float`.
Repeated subtraction from a very large float can stagnate. Do not simulate billions of no-op
substeps; preserve the normal-step behavior and consume remaining time safely when agents are
quiescent.

Add deterministic regression tests for the denormal-duration and half-to-one-ULP far-origin
cases, including finiteness, containment, observed speed, and unchanged state where no
representable legal movement exists.

## 2. Repair local avoidance so feasible agents remain separated *and* progress

The current snapshot intent is sound, but the final collision correction followed by individual
mesh clipping does not guarantee the actually committed endpoints remain separated. The close
reflex-corner fixture falls below the required `radius sum - 1e-3` threshold (observed about
`0.49849` for a required `0.499`) and the follower then stalls near the inside corner.

The ordinary open-space crossing fixture also does not complete: after 12 seconds both agents
remain `Moving`, approximately at `x=7.979`, with centres repeatedly constrained at or near
exactly `0.5` separation rather than entering their distinct arrival radii. This violates
**SIM-010**, **SIM-011**, and **SIM-012**.

For every internal substep, base all decisions on one immutable position/velocity snapshot.
Evaluate the **actual feasible stored motions**—after containment limitation and float storage
rounding—when enforcing pair separation. Select deterministic local candidates that retain a
valid route/progress option (desired, yield/reduced, and stable tangential/separating choices
are sufficient); do not make a sequential update or positional push. A candidate that only
preserves separation by indefinitely stopping a feasible agent is not adequate. Keep all normal
agents finite, continuously contained, actually speed-bounded, and separated within the stated
tolerance. The two agents in the supplied crossing, overtaking, and close reflex-corner following
situations must reach their distinct goals. Initially overlapping agents still need a deterministic
finite contained separation attempt.

No named avoidance algorithm, global planner, or arbitrary-density guarantee is required. Do
not relax the separation tolerance or snap/teleport an agent to its goal.

Add focused deterministic tests for crossing completion, reflex-corner following completion,
and the tight post-step separation check. Keep the existing tests; assertions must check public
state/invariants rather than a private exact trajectory.

## 3. Reduce the concentrated policy complexity while making the repair

The architecture has a sensible Geometry → immutable Mesh/Path → mutable Simulation dependency
flow, clear pImpl ownership, and no scope violations. Preserve that. However,
`NavMesh::create` is a 123-line, clang-tidy cognitive-complexity-92 routine, and the simulation
helper currently mixes steering, predicted overlap correction, containment bisection, integration,
and route-status updates. Extract only cohesive private helpers that make the numeric and
avoidance invariants independently understandable and testable; do not introduce framework-like
abstractions or duplicate geometry rules. In particular, name the difference between intended
velocity, mesh-feasible displacement, stored endpoint, and committed velocity.

Update the architecture/implementation plan after the code and tests pass to describe the real
stepping/avoidance boundary and the repair. Do not claim a verification result that was not run.

## 4. Strengthen ordinary project quality evidence

The candidate has one small custom assertion executable (145 lines) and misses every reported
numeric/crowd defect. Retain it or replace it only with an equally clear CTest-integrated test
setup, and extend coverage across all repaired success and failure paths. Build tests must remain
fast and deterministic.

NFR-007 requires consistently formatted source. The tree has no checked formatting configuration
and `clang-format --dry-run --Werror` with the available default style reports every source/header
file. Establish and apply one checked project style (without changing the fixed public API), then
make the full submitted C++ tree pass the corresponding clang-format check.

Before completion, run and report actual outcomes for:

1. clean Debug and Release GCC builds/tests with `-Wall -Wextra -Wpedantic`;
2. clean Clang build/tests with the same warnings;
3. ASan/UBSan tests if available;
4. the supplied public conformance runner when it is available in your workspace; otherwise
   preserve/add the equivalent public-contract regressions without copying evaluator material;
5. a `BUILD_TESTING=OFF` library-only configure/build; and
6. clang-format check mode over all submitted C++ sources and headers, excluding generated
   build trees.

Preserve all fixed signatures, defaults, enums, error meanings, include paths, and the
`vwmini::vwmini` target. Do not modify evaluator/reference material.
