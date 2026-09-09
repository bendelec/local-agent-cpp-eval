# Final Repair Follow-up

Continue from the first repaired VWmini library. This is a **small, final, focused** repair. Preserve the fixed public API, the now-passing geometry/navmesh/lifecycle/storage behavior, CMake test gating, deterministic snapshot-based stepping, and the successful crossing, overtaking, overlap-recovery, and tight-corridor cases. Do not replace the project wholesale; remove or weaken tests; hard-code coordinates, meshes, routes, agent IDs, or scenario-specific behavior; add third-party dependencies, global mutable state, or a global crowd planner.

The first repair correctly fixed the finite-vector, stored-float speed-bound, library-only CMake, and identifier-exhaustion issues. GCC and Clang ASan/UBSan candidate tests pass, library-only configuration now works, and conformance now passes **81/82**. The sole remaining failing normative case is a general local-avoidance progress failure.

## Restore close-following progress around a reflex corner

The normative scenario is a one-metre-wide L passage: bottom arm `[0,5] x [0,1]`, left arm `[0,1] x [1,5]`; two radius-`0.25`, speed-`1.4` agents start at `{2.5,0.5}` and `{3.1,0.5}` and have distinct goals `{0.5,3.0}` and `{0.5,4.0}`, respectively, with arrival radius `0.10`. They begin already separated. After 600 calls to `step(1/60)`, the leader reaches but the follower remains `Moving` with zero velocity, stranded near the leader in the vertical arm.

The prior change that omits a speculative moving-neighbor margin for a `Reached` neighbor was sound for the tight straight corridor, but it did not solve this local minimum. The current scorer ranks feasible candidates solely by instantaneous dot-product progress toward the desired route velocity. Since zero velocity is a feasible candidate and every safe immediate escape/tangential/retreat candidate has negative route progress, the stateless replan selects zero forever.

Fix this as a **general deterministic local recovery policy**, not as a condition keyed to this mesh, position, agent order, goal, or test. For example, a blocked moving agent can select a deterministic contained escape/tangent direction and retain that choice only until it regains a collision-safe route-progress motion. Another small route-aware policy is acceptable. The essential properties are:

- a feasible close follower must retain or regain forward route progress instead of remaining indefinitely `Moving` at zero velocity;
- every committed stored endpoint remains finite, continuously in the mesh, and within its real stored-position speed budget;
- agents that were initially separated never get closer than the required radius sum (with the documented `1e-3` tolerance);
- decisions stay deterministic and are planned from the frozen substep snapshot; and
- non-moving agents remain stationary and are not given a hypothetical motion that differs from commit.

Do not relax the separation floor, teleport an agent, turn a reachable route into `NoPath`, or trade this case for failures in the existing crossing, overtaking, overlap-recovery, narrow-corridor containment, replay, or tight-corridor tests. Keep the correction local and understandable; a tiny per-agent recovery state is acceptable if it has a clear lifecycle and reset conditions.

## Add the missing regression and correct the documentation

The first repair's `FollowerPassesReachedLeaderInTightCorridor` is a useful straight-corridor test, but it is not the requested reflex-corner close-following regression and passes while the normative L scenario fails. Add a focused candidate test matching the L-passage *class* above through the public API. On every step, check both agents for finite positions/velocities, mesh containment, actual reported speed, and separation. Assert that both reach within the 600-step budget. Retain the existing test; do not merely substitute it.

Update the architecture and implementation-plan documents after the implementation works. Distinguish the already-fixed immobile-neighbor margin from the new blocked-agent progress/recovery rule; do not claim that the straight-corridor regression covered the reflex case before it did.

Before completion, run and report actual outcomes for:

1. clean GCC Debug and Release builds/tests with `-Wall -Wextra -Wpedantic`;
2. a clean Clang build/test with the same warnings;
3. Clang ASan/UBSan tests if available;
4. a `BUILD_TESTING=OFF` library-only configure/build without GTest; and
5. clang-format check mode over submitted C++ sources and headers, excluding generated build trees.
