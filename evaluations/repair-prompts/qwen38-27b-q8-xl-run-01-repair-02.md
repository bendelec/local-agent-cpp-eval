# VWmini Second Repair Pass

Continue work in the current VWmini workspace. Preserve the supplied public API,
all existing requirements, and the completed repairs already present in this
workspace. Do not replace or weaken existing regression scenarios.

Three robustness and progress issues require completion before final evaluation.
Generalize the solution; do not hard-code a fixture coordinate or agent id.

## 1. Large finite `step()` duration must remain safe

`Simulation::step(float seconds)` accepts every finite non-negative duration.
The internal substep-count cap can leave a very large finite `dt`. In an
avoidance case, waypoint exact-landing may be cancelled, then integrating a
speed-bounded velocity by that huge `dt` can overflow to infinity. The
containment bisection must never turn that into a NaN position (for example,
zero times infinity), and every agent must remain finite and contained.

Repair this generally and directly. Keep deterministic behavior, snapshot-based
avoidance decisions, containment, speed bounds, and ordinary-duration progress
intact. Do not solve it by rejecting valid large finite durations. A suitable
approach may revise subdivision/integration or add a finite safe fallback, but
must preserve the contract for valid input.

Add a focused deterministic native regression using an initially overlapping
pair with non-unit/default-like maximum speeds and
`std::numeric_limits<float>::max()` (or an equivalently demanding finite value).
After the call, assert success, finite position and velocity, and containment
for every state. The fixture must exercise avoidance that cancels an exact
landing; a lone exact-landing agent is insufficient.

## 2. Close-following agents must progress through a reflex corner

A valid single-agent bent route can still fail in practice when a second agent
follows closely in the same direction. Two non-overlapping discs on the same
connected L-shaped route can enter each other's local avoidance influence; the
leading agent must not become permanently stuck at the reflex corner merely
because the follower is behind it. This is not an infeasible shared-goal crowd
case: use distinct terminal positions with enough separation for both discs to
occupy their goals.

Diagnose the interaction among path waypoints, avoidance steering, containment,
waypoint advancement, and public step duration. Preserve simultaneous snapshot
decisions, determinism, finite/contained state, speed limits, and normal
crossing/overtaking behavior. Do not disable avoidance, exempt nearby agents,
or special-case a coordinate.

Add a deterministic native regression using the L-shaped union of bottom arm
`[0,5] × [0,1]` and left arm `[0,1] × [1,5]`, with its inner/reflex corner at
`(1,1)`. Use radius-0.25, max-speed-1.4 agents at `(2.5,0.5)` and `(3.1,0.5)`
(the initial centre distance is 0.6, greater than the 0.5 touching distance but
within local-avoidance influence). Give them the distinct goals `(0.5,3.0)` and
`(0.5,4.0)`, whose centres are separated by 1.0. Step at `1/60 s` for at most
600 calls. After every step assert finite state, containment, velocity magnitude
no greater than `max_speed + 1e-4`, and centre separation at least
`radius_a + radius_b - 1e-3`; assert both reach their goals by the deadline.

## 3. Regression and documentation integrity

- Restore the pre-existing overtaking and angled-crossing avoidance scenarios
  unchanged. If additional variants are useful, add them as distinct tests.
- Correct comments/documentation that describe a 30 Hz public call as three
  internal 0.1-second substeps: the present subdivision calculation uses one
  short substep for that duration.
- Retain all existing tests, including the single-agent reflex-corner and
  overlap-recovery regressions, and rerun them.

## Verification before completion

Build from scratch with GCC and Clang using `-Wall -Wextra -Wpedantic`; run
CTest and an ASan/UBSan build if available. Update the architecture,
implementation plan, and completion report to describe the actual changes,
their causes, regression coverage, and verification results. Keep the solution
small and local; do not add framework code, global state, or a crowd-planning
layer.
