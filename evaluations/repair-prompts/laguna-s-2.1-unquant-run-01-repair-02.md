# VWmini Follow-up: Compatible Crowd Motion and Duration Semantics

Continue work in the current VWmini workspace. Preserve the supplied public API and all
documented requirements. The prior repair correctly resolved mesh/path and lifecycle
behavior. Do not rewrite those working layers, relax or remove existing tests, or
hard-code a scenario, coordinate, agent ID, mesh layout, or trajectory.

## 1. Replace projection-only avoidance with compatible local choices

The remaining crowd behavior is caused by the current avoidance loop adjusting only the
relative velocity along each pair's separation normal. It can cancel goal-directed
motion, make a faster follower match the leader's speed, or leave close agents outside a
waypoint threshold indefinitely. The code currently has no actual candidate-selection
phase despite describing one in documentation.

Keep decisions deterministic, local, and snapshot-based, but choose velocities that are
compatible for the actual simultaneous outcome. A small bounded candidate set is
sufficient: desired motion, reduced/yielding motion, stop, and deterministic tangential
sidesteps. Evaluate candidate pairs using predicted swept-disc separation over the
substep, speed limits, and mesh containment. Rank feasible choices by retained progress
toward the active waypoint/goal, with a stable index-based tie-break. Revalidate affected
neighbours before committing all selected velocities simultaneously.

This must repair general ordinary feasible behavior, not merely these examples:

- Two agents on crossing open-space routes remain separated within the documented
tolerance and both reach their distinct valid goals.
- A faster agent can overtake a slower same-direction agent without overlap, and both
reach their goals.
- Close agents following a feasible reflex-corner route remain contained and separated,
then both reach their distinct goals.
- Initially coincident agents still make a deterministic, finite, contained separation
attempt.

Do not add global planning, mutable global state, a crowd framework, or fixture-specific
priority rules.

Add focused deterministic native regressions for crossing, overtaking, and close
reflex-corner following. After *every* step, assert speed bounds, finite contained state,
and disc separation; at the deadline assert `Reached` for every feasible agent. Retain
the existing coincident-centre regression.

## 2. Preserve the full valid step duration safely

The current large-duration guard silently truncates any requested duration above 1000
seconds. `step(seconds)` must advance for the finite duration it accepts, not return
success after simulating a smaller interval.

Bound computational work without discarding elapsed time: safely bound the substep
*count* and use the requested duration to determine the resulting substep duration, or
use another numerically safe chunking strategy. Avoid unsafe floating-to-integer
conversion for values such as `FLT_MAX`, avoid unbounded loops, and preserve finite,
contained, speed-bounded motion. Add a regression that distinguishes a duration greater
than the former cap from exactly the cap on a sufficiently long valid route.

## 3. Documentation integrity and final verification

Correct documentation to match the repaired implementation. In particular, do not claim
candidate-based avoidance or deadlock-free ordinary crowd behavior unless the submitted
algorithm and tests establish it. Record the true large-duration strategy and the new
regressions.

Retain the public declarations and their semantics exactly. Formatting-only consistency
changes are acceptable only if no declaration, signature, default, enum value, include
contract, ABI-relevant layout, or linkage behavior changes.

Before completion, build from scratch with GCC and Clang under `-Wall -Wextra
-Wpedantic`, run the full native suite, run ASan/UBSan if available, and run
clang-format in check mode. Update the architecture document, implementation plan, and
completion report with only checks actually run and their results.
