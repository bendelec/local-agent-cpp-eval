# Fix blocked local avoidance in an inside corner

Fix a blocked local-avoidance issue in the VWmini library. Keep the public API unchanged and preserve the existing geometry, navmesh, lifecycle, numeric movement, build, and crowd behavior. Make a small, targeted correction; do not replace the project wholesale, remove or weaken tests, hard-code particular coordinates, meshes, routes, agent IDs, or trajectories, add dependencies, introduce global mutable state, or add a global crowd planner.

## Problem report

A close follower can become permanently stuck while rounding an inside corner behind an agent that has already reached a nearby, distinct goal. The follower remains `Moving` but reports zero velocity indefinitely even though there is a valid route and enough room to pass without disc overlap.

The issue is reproducible in a one-metre-wide L passage with a horizontal bottom arm `[0,5] x [0,1]` and vertical left arm `[0,1] x [1,5]`. Two radius-`0.25`, speed-`1.4` agents start at `{2.5,0.5}` and `{3.1,0.5}`. Their distinct goals are `{0.5,3.0}` and `{0.5,4.0}`, both with arrival radius `0.10`. The first agent reaches, but the follower can stop near it in the vertical arm instead of reaching its own goal.

The fixed velocity sampler scores feasible choices only by immediate alignment with the desired route velocity. A zero-velocity fallback can therefore beat every safe tangential or short retreat choice when those choices initially have negative route alignment. Repeating this stateless choice creates a local minimum.

## Required behavior

Implement a general, deterministic local recovery rule for a blocked moving agent. A small per-agent recovery/yield state is acceptable if it has a clear lifecycle and reset condition. For example, when no safe forward candidate exists, select a deterministic contained escape or tangent direction and retain it only until a collision-safe, route-progressing choice is available again. Other small route-aware solutions are welcome.

The correction must preserve these invariants:

- a follower in a feasible passage regains route progress rather than staying `Moving` with zero velocity indefinitely;
- committed stored positions and velocities remain finite and obey the actual stored-position speed bound;
- motion stays continuously within the navmesh;
- agents that start separated preserve disc separation to the documented `1e-3` tolerance;
- planning remains deterministic and based on the frozen substep snapshot; and
- Idle, Reached, and NoPath agents remain stationary rather than receiving an avoidance motion that is not committed.

Do not relax separation, teleport an agent, convert a reachable goal to `NoPath`, or regress the established crossing, overtaking, initially-overlapping-agent recovery, narrow-corridor containment, deterministic replay, or tight-corridor behavior.

## Tests and documentation

Add a concise public-API regression test for the L-passage close-following case described above. At each step, check finite positions and velocities, mesh containment, reported speed, and pair separation; verify both agents reach within ten seconds. Keep the existing straight tight-corridor test as separate coverage.

Update the architecture and implementation-plan documentation once the implementation is complete. Describe the actual blocked-agent recovery rule and distinguish it from the existing treatment of stationary neighbors in narrow corridors. Keep claims factual.

Before finishing, run the project’s Debug and Release GCC test builds with warnings enabled, a Clang test build, sanitizer tests if available, a library-only `BUILD_TESTING=OFF` build without GTest, and clang-format check mode over submitted C++ sources and headers. Report the commands and actual results.
