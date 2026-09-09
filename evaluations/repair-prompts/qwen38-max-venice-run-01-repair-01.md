# Repair Follow-up

Continue improving the current VWmini library. Preserve the supplied public API, requirements, and
behaviour that already works. Make a focused repair; do not replace the project wholesale, weaken
or delete tests, hard-code a coordinate, mesh, agent id, route, or scenario-specific trajectory,
or add third-party dependencies, global mutable state, or a global crowd planner.

The project has a strong modular structure and broad deterministic test suite. The issues below are
valid-input boundary and progress defects in otherwise sound policies. Address their general causes
and add concise regression tests through the existing GTest/CTest setup.

## 1. Make finite vector operations and stored movement numerically honest

`length({FLT_MAX, FLT_MAX})` currently becomes infinity and `normalized` returns `{0,0}` even
though the input is a non-zero finite vector. The fixed public contract requires finite input to
produce a finite non-negative length. Use a scale-safe/double-precision calculation; where the true
length cannot be represented as `float`, choose and document a finite saturation policy. Normalize
from the scaled or widened value rather than from an overflowed float length. Cover both maximum
finite and tiny non-zero vectors.

Motion has a related storage-boundary issue. A finite agent at `{10000, 0}` with maximum speed `1`
and a `0.0006f` step can be stored one float ULP farther away, `0.0009765625`, than its movement
budget. Plan in widened arithmetic if useful, but validate the **stored float endpoint** and velocity
against the budget. If no representable progress is legal, a finite no-op with zero velocity is the
right outcome; do not retain hidden residual distance for a later teleport. Test actual observed
stored displacement and reported velocity, not only intended double arithmetic.

## 2. Restore progress for a feasible close follower at a reflex corner

The local candidate policy keeps agents separated in ordinary crossing and overtaking cases, but a
close follower can stop permanently behind an already-reached agent in a one-metre-wide L passage.
The scene is feasible: two radius-0.25 agents, speed 1.4, distinct goals around the same corner, and
an arrival radius of 0.1. The follower must not remain `Moving` with zero velocity after a generous
budget merely because the leader selected an early terminal position.

Retain simultaneous snapshot planning, continuous mesh containment, real stored speed bounds, and
the separation tolerance. Improve the general local selection/arrival policy so its committed
motions preserve a feasible route/progress option for a nearby follower. A small deterministic
route-aware yield, tangential, reduced-speed, or terminal-placement choice is sufficient; a global
planner is not required. Do not relax tolerances, overlap the discs, or teleport an agent to its
goal.

Add this close-following L-passage regression with per-step public checks for finite state,
containment, observed speed, separation, and both terminal statuses. Keep the existing crossing,
overtaking, initial-overlap, and replay tests.

## 3. Complete library configuration and lifetime edge cases

A consumer building only `vwmini` must not require GTest. Use normal CMake test gating (for example,
`include(CTest)` and `BUILD_TESTING`) so a library-only configure/build succeeds when no test
package is available; keep the default project test experience intact.

Agent identifiers must never be reused for one `Simulation` lifetime. Do not wrap the monotonic
32-bit identifier back to `1`; when the space is exhausted, return the existing `InvalidArgument`
error category before mutating slots, the id map, free-slot state, or the next-id state. Isolate the
allocation decision enough to test that no-mutation exhaustion outcome without billions of public
calls if necessary, while keeping the public API unchanged.

## 4. Keep the repair small and documentation accurate

The module boundaries, pImpl ownership, internal double-precision predicates, and focused GTest
layout are strengths to preserve. Avoid introducing a second geometry policy or expanding the
sampling mechanism into a framework. The `step` documentation currently says substeps are always
at most 1/60 seconds, but its existing finite cap makes larger substeps for very long durations.
Document the real behaviour and its invariant, or revise the implementation to make the statement
true.

Update the architecture document and implementation plan only after the implementation and tests
pass, describing the actual endpoint-budget, crowd-progress, identifier, and test-build policies.
Do not claim verification that was not run.

Before completion, run and report actual outcomes for:

1. clean Debug and Release GCC builds/tests with `-Wall -Wextra -Wpedantic`;
2. a clean Clang build/test with the same warnings;
3. ASan/UBSan tests if available;
4. a `BUILD_TESTING=OFF` library-only configure/build without GTest; and
5. clang-format check mode over submitted C++ sources and headers, excluding generated build trees.
