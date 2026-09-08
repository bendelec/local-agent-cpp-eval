# Repair Follow-up

Continue in the current VWmini workspace. Preserve the supplied public API, requirements,
and the substantial behavior that already works. Do not replace files wholesale, weaken or
delete tests, hard-code an input coordinate, mesh, agent id, route, or fixture-specific
trajectory, or add third-party dependencies, global mutable state, or a global crowd planner.

The initial implementation is a strong, modular baseline: native Debug/Release tests,
warning-clean GCC/Clang builds, sanitizer tests, formatting, consumer linkage, path behavior,
and all small feasible crowd cases are in good shape. Independent public-API checking finds
one remaining normative failure, caused by a broader numerical-policy gap. Address the policy,
not only its observed triangle.

## 1. Make accepted finite geometry scale-safe end to end

A valid finite extreme-coordinate triangle is accepted but reports its strictly interior
point as outside. Internal functions named `signed_double_area`, `ring_double_area`,
`length_squared`, segment-distance/projection, and orientation/containment predicates still
perform float multiplication, subtraction, or accumulation. Those operations overflow or
lose the needed sign at accepted finite scales. The contract requires signed **double** area
for triangle degeneracy, and finite mesh coordinates must remain usable for containment,
endpoint location, and paths.

Centralize an explicit internal numeric policy using scale-safe double intermediates wherever
geometry decides validity, incidence, containment, segment coverage, topology, or a route.
Do not change the fixed public declarations or redefine the public `dot`/`cross` signatures.
Use robust internal helpers instead. Preserve the documented epsilon rule: it applies only to
point/edge distance and complete-edge endpoint matching, not as a general orientation or
area tolerance. Avoid treating overflow, infinity, or a failed calculation as evidence that
finite input is outside or invalid.

The public header also promises a finite non-negative `length` for finite `Vec2` input.
`length({FLT_MAX, FLT_MAX})` currently becomes infinity, and `normalized` becomes `{0,0}`
for that nonzero finite vector. Keep ordinary values accurate; for an unrepresentable true
float norm, return a finite documented/sensible result and compute normalization by scaling so
that a nonzero finite vector does not collapse to zero. Apply the same care to accepted finite
agent radii/speeds and pairwise physical arithmetic: valid finite input must not create
non-finite state, overflow-driven false collision decisions, or invalid movement.

Add deterministic GTest regressions that remain active in Release for:

- a finite extreme-coordinate CCW triangle whose interior point is contained;
- finite extreme `length`/`normalized` behavior;
- the repaired containment/path behavior on such geometry, where representable;
- ordinary-scale topology, boundary tolerance, direct paths, and every existing crowd case,
  so the numeric change does not regress the already passing implementation.

## 2. Honor the standard library-only CMake configuration

The project currently exposes `VWMINI_BUILD_TESTS`, but configuring with
`-DBUILD_TESTING=OFF` emits an unused-variable warning and still builds `vwmini_tests`.
Honor standard `BUILD_TESTING` so a library-only configuration builds only the library and
has no test target. Compatibility with the existing project-specific option is fine if it is
clear and cannot produce contradictory settings. Retain CTest/GTest tests by default and the
downstream-consumer check.

## 3. Keep documentation and public-header claims precise

The implementation report says the public headers are exactly as supplied except for one
private friend declaration, but they also contain broad formatting-only edits. Preserve every
public declaration, signature, default, enum value, include path, and target name; do not make
further public-surface edits merely for style. Update the report, architecture, plan, README,
and scripts only after verification so they accurately describe the actual numeric policy,
`BUILD_TESTING` behavior, public-header status, tests, and commands. Do not claim a test or
configuration is clean unless it was actually run.

Before completion, run and report only actual outcomes for: native Debug and Release tests;
warning-clean GCC and Clang builds with `-Wall -Wextra -Wpedantic`; an ASan/UBSan test build
if available; `-DBUILD_TESTING=OFF` library-only configuration; the downstream consumer; and
a formatting check over source and tests. Keep the architecture modular and direct—this is a
focused numerical/build repair, not an invitation to redesign the working pathfinding or
crowd system.
