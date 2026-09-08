# Final Repair Follow-up

Continue in the current VWmini workspace. This is the final repair. Preserve the successful
scale-safe double-precision geometry/topology/path repair, the public API, all requirements,
the CMake `BUILD_TESTING` correction, and the currently passing behavior. Do not weaken/delete
tests, hard-code an input/mesh/agent/trajectory, add a third-party dependency, add global mutable
state, or redesign the working pathfinding or local-avoidance algorithm.

The last repair solved the original finite-extreme geometry defect and now passes the full public
suite, but it also added unsolicited motion-policy mechanisms that were neither needed to repair
that defect nor justified by a stated requirement. Those extensions introduced reproducible
contract violations and an unsafe conversion path. Scope discipline matters: complexity that does
not directly and correctly satisfy a requirement is a regression, not extra credit. Fix the
defects directly and simplify; do not broaden the system again.

## 1. Never accept a motion segment from endpoint-plus-midpoint sampling

`Simulation` now accepts a displacement when only its start, endpoint, and one midpoint are in
the mesh. The architecture documentation itself admits that a thin obstacle can still be crossed.
That is not sufficient for SIM-012: motion must not teleport through non-walkable space.

A concrete public-API regression is reproducible with a mesh containing these disjoint regions:

- a vertical rectangle spanning `x=[-1,1]`, `y=[-1,151]`;
- a small triangle containing `(50,0)`;
- a small triangle containing `(100,0)`.

Put a moving agent at `(0,0)`, radius `0.25`, speed `100000`, goal `(0,150)`, and an idle agent
at `(-0.5,0)`, radius `0.25`. During `step(0.001)`, overlap recovery drives the moving agent
right. Start, `(50,0)`, and `(100,0)` are contained, while `(25,0)` and `(75,0)` are not. The
current midpoint check moves the agent to `(100,0)`, crossing two gaps.

Replace this approximation with the existing exact segment-coverage logic already used for
routing (`MeshTopology::segment_is_contained`), exposed through the narrowest private internal
NavMesh seam needed. Do not expand the public API merely for this. Every accepted displacement
must have a completely contained stored-position segment. Add a deterministic Release-active
GTest for the scenario above that proves the agent does not cross either gap, while retaining all
ordinary route and crowd cases.

## 2. Remove or correctly bound residual-motion state

The new `AgentRuntime::residual` silently accumulates sub-ULP motion and later rounds the logical
double position to the nearest `float`. It can make observable movement exceed the max-speed
budget. Reproduction: on a valid square near `x=10000`, an agent with `max_speed=1`, moving east,
advanced five times with `step(0.0001)`, jumps by one float ULP (`0.0009765625`) on call five,
although the whole elapsed allowance is only `0.0005`.

The simplest correct solution is to remove residual accumulation: a public `Vec2` position that
cannot represent a displacement may remain unchanged. If retaining any residual mechanism,
prove instead that every call and every cumulative interval respects `distance <= max_speed *
elapsed_time`, that it is reset on every goal/route/idle transition, and that it cannot inject
stale lateral movement after `set_goal` or `clear_goal`. Prefer the simpler design unless the
more complex one is demonstrably needed by a stated requirement.

Add a deterministic far-origin small-step regression that checks per-step and cumulative speed
bounds using double arithmetic. Keep the useful representable extreme-distance steering
regression, but do not require invisible sub-ULP travel to be accumulated.

## 3. Check range before narrowing double positions to `Vec2`

`Vec2d::to_vec2()` currently performs an unconditional `double` to `float` cast. Motion calls it
before checking whether the landing position is finite. Valid finite configurations can produce a
double candidate outside `[-FLT_MAX, FLT_MAX]` (for example, a large finite step plus an
overlap-driven deflection away from the goal), so the guard comes too late.

Make position narrowing explicitly checked before any conversion: finite and within the
representable float range, returning an optional/result/bool as appropriate. On failure, reject
or shorten the candidate without creating non-finite state or relying on implementation-defined
or undefined conversion behavior. Keep value construction ergonomic for known-representable
geometry, but do not leave a generally unsafe `to_vec2()` call site. Add a finite extreme
speed/duration/deflection regression that verifies finite state, speed cap, and containment.

## 4. Remove the scope creep; keep only necessary, correct behavior

The central `Vec2d` numeric policy, double predicates, extreme containment/path tests, and
standard CMake testing option are justified and should remain. In contrast, the midpoint sweep
and residual integrator were unnecessary extensions beyond the requested numerical/build repair,
and they made the implementation observably less correct. Do not preserve them merely because
they were already written.

Use the smallest design that meets the contract:

- replace midpoint sampling with the existing exact containment primitive, not a new movement
  subsystem;
- remove residual state unless it can meet every speed/transition requirement proved by focused
  tests; and
- do not add any other proactive hardening, algorithm replacement, policy layer, or “review
  round” feature outside the three concrete defects above.

A clean deletion is preferable to a speculative abstraction. Do not add unrelated features.

Update tests, architecture, implementation plan, README, and implementation report only to
reflect code actually retained and commands actually run. In particular, do not describe a
midpoint sample as preventing all out-of-mesh motion. Preserve the public declarations,
signatures, defaults, enum values, include paths, and library target.

Before completion, run actual native Debug and Release tests; warning-clean GCC and Clang builds
with `-Wall -Wextra -Wpedantic`; ASan/UBSan if available; full public conformance; a
`-DBUILD_TESTING=OFF` library-only build; downstream consumer; and a formatting check. Report
only those real outcomes.
