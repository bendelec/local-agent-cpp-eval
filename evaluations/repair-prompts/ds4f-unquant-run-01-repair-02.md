# VWmini Second Repair Pass

Continue from the completed first-repair workspace. Preserve the supplied public API,
all requirements, and all existing regressions. Do not replace tests, relax assertions,
or hard-code a coordinate, agent ID, mesh shape, or test-specific trajectory.

Independent verification found two concrete defects in the new avoidance and large-
duration handling, plus the existing crowd conformance failures. Repair the general
algorithms, not just these examples.

## 1. Preserve separation for all same-direction neighbours

The new same-lane side-pinning rule can reduce separation for agents that are already
almost abreast but laterally offset. For example, two radius-0.25 agents moving east,
at approximately `(10,10)` and `(9.99,10.51)`, with speeds `0.1` and `0.2`, become
closer than their combined radius after one short step even though free space exists.

The leader/follower decision must have a robust longitudinal-order condition. When
agents are abreast or laterally separated, the selected corrections must not steer them
toward one another. More generally, preserve the required per-step disc separation for
crossing, overtaking, close following, and initially overlapping recovery while retaining
simultaneous snapshot decisions, determinism, containment, speed limits, and progress.

Add a focused deterministic native regression for the near-abreast same-direction case,
checking separation after every step and eventual feasible progress where appropriate.
Do not weaken the existing crossing or reflex-corner regressions.

## 2. Every finite `step()` duration must make safe progress

The substep cap avoids an unbounded loop, but it can make `dt` so large that calculating
`distance / dt` in `float` underflows to zero. A valid moving agent with zero arrival
radius and a very close non-identical goal can consequently remain `Moving` forever
when stepped with `std::numeric_limits<float>::max()`.

Repair the general integration/arrival calculation using numerically safe arithmetic or
a direct safe landing rule. Valid finite input must not be rejected. Preserve finite and
contained positions and velocities, speed bounds, exact terminal-state behavior, and
ordinary-duration movement. Add a native regression that exercises this underflow case
and verifies the agent reaches its valid goal.

## 3. Bound encounter-state lifetime

The per-pair same-lane state must not accumulate after agents are removed or after an
encounter becomes irrelevant (including both agents becoming non-moving). Clean it up
without changing public IDs or snapshot semantics. Add a small regression if practical.

## 4. Complete the existing public crowd behavior

The current implementation still fails the ordinary open-space crossing scenario and the
close-following L-shaped reflex-corner scenario. Diagnose and repair their actual shared
steering/containment/path interaction. Both agents in each feasible scenario must remain
finite, contained, speed-bounded, and separated at every step, then reach their distinct
goals by the stated deadline. Do not disable avoidance, exempt close agents, or add a
crowd-planning framework.

## Documentation integrity and verification

Correct stale documentation so it describes the delivered search algorithm accurately:
the corridor search is uniform-cost Dijkstra, not A*, and it does not optimize a
centroid-distance objective. Correct stale native-test-count claims in the implementation
plan/completion report. Keep documentation specific and truthful.

Retain all prior native tests and add the focused regressions above. Build from scratch
with GCC and Clang under `-Wall -Wextra -Wpedantic`, run CTest, and run ASan/UBSan if
available. Update the architecture document, implementation plan, and completion report
to reflect the actual changes and verification. Keep the repair local, direct, and
readable; do not add global state or fixture-specific logic.
