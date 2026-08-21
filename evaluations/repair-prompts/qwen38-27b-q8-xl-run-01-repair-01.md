# Repair Feedback — Qwen 3.8 First Attempt

Your first implementation has been archived immutably as
`solutions/qwen38-27b-q8-xl-run-01/`. Continue work only in the live workspace
`../vwmini_q38/`; do **not** modify the archive, evaluator, public headers, task
requirements, or supplied tests.

A subsequent visual integration review found two reproducible functional defects. The
original public suite passed, but the submission is not ready without these repairs.
Do not hard-code any particular scenario or coordinate.

## Required repairs

1. **Single-agent path following through a reflex corner can stall.**
   A valid contained path around an L-shaped mesh's inner corner is returned correctly,
   but a moving agent can fail to reach its valid in-mesh goal. The failure depends on
   the public `step(seconds)` duration: a normal 30 Hz caller can leave the agent
   clamped at the reflex corner while it continues to report `Moving`.

   Diagnose the interaction among waypoint completion, route-index advancement,
   containment, and public substep duration. A waypoint must not be considered safely
   passed merely because doing so makes the following steering segment cut through
   non-walkable space. Preserve SIM-008/SIM-009: every state remains contained and a
   single agent with a connected route makes progress to `Reached`, without speed-limit
   violations or overshoot, for ordinary finite public step durations.

2. **Initially overlapping discs do not recover separation.**
   In an otherwise open convex mesh, two live agents whose centres start overlapping
   can remain overlapped or be driven together. This violates SIM-010's explicit edge
   case: remain finite and contained **and attempt separation**. Carefully audit the
   relative-position convention and direction/sign used by the overlap response. The
   solution must make a material separation recovery in this simple two-agent case,
   while retaining simultaneous snapshot decisions, determinism, maximum-speed bounds,
   containment, and the already-passing crossing/overtaking behavior.

Do not treat multiple discs assigned the exact same terminal goal as a required global
crowd-solver problem: collision-free occupancy of one identical point is infeasible and
outside the requested repair. Focus on the two defects above.

## Verification expected before completion

Add focused deterministic native regressions that would have caught both failures:

- A single connected bent/reflex-corner route reaches its goal under a normal public
  frame duration, with containment and speed checked after every step.
- An initially overlapping pair in open space becomes materially more separated; do
  not call it a separation test while disabling or weakening the separation assertion.

Retain and rerun all existing tests. Build from scratch with GCC and Clang under
`-Wall -Wextra -Wpedantic`; run CTest and an ASan/UBSan build if available. Update
`docs/architecture/architecture.md`, `implementation-plan.md`, and the completion report
to describe the actual repair, its cause, regression coverage, and any revised design
choice. Keep the design direct; do not add a framework, global state, or unnecessary
crowd-planning layer.
