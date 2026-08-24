# VWmini — Completion Report

Status: **complete** — all 20 slices S1.1–S7.3 of
`implementation-plan.md` done; 76/76 tests green and warning-clean from a
clean configure on both g++ and clang++; ASan/UBSan clean. After the
completion report was first written, an independent review found two
behavioural defects (reflex-corner stall; overlapping-disc
non-separation); both are fixed, regression-tested, and documented in the
**Review defect fixes** section below. A second review round found three
more issues (large finite `step()` durations overflowing to NaN; a
close-following pair stalling at a reflex corner; test/doc integrity);
all three are fixed, regression-tested, and documented in the
**Second review round** section below.

## Design & modules

A single C++23 static library, with strictly inward dependencies and value
semantics throughout (see `architecture.md`, kept current throughout,
including the Mermaid module diagram):

| Module | Responsibility / why this shape |
|---|---|
| `geometry_priv` | Pure predicates: finiteness, orientation/area, point-to-segment distance, strict triangle interior, segment intersection. The shared numerical core; everything else builds on it. |
| `triangulate` | `triangulate_simple_polygon` (MSH-002/003): full outline validation + deterministic ear clipping. Chosen over incremental triangulation because MSH-002 restricts input to simple CCW hole-free polygons — ear clipping with overlap checks is simpler and adequate (NFR-008 forbids scope growth). |
| `mesh_priv` + `nav_mesh.cpp` | `NavMesh::create` validation (MSH-004: CW/degenerate/overlap/T-junction/non-manifold), shared-edge adjacency (MSH-005), and `contains` (MSH-006) as a brute-force scan — const, allocation-free, and NFR-008 sets no performance target, so a spatial grid was dropped as unjustified complexity (recorded revision). `NavMesh` holds `shared_ptr<const Impl>` (shape fixed by the supplied header) for cheap value copies. |
| `path` | `find_path` (SIM-001…004): endpoint validation, direct-segment shortcut, Dijkstra over the adjacency graph with deterministic (distance, cell index) tie-break, then **iterative string-pulling** over shared-edge portals (coordinate descent; jointly convex objective ⇒ coordinate-wise optimum is the funnel/string-pulling result SIM-003 requires — no unshortened cell-centre route). Output is refined (ε-omission, corner snapping) and then gated by an **exact per-segment MSH-006 interval containment check**; any violation returns `NoPath`, making SIM-002's "every real t" a checked invariant. |
| `simulation` (+ `simulation_impl.hpp`) | Public `Simulation` PIMPL. Slot-indexed `std::vector<SimAgent>` (slot i == `AgentId{i+1}`), monotonic non-reused ids, `live_count`. Goal state machine shared by `add_agent`/`set_goal` (Moving / immediate Reached / NoPath-as-success). `step` runs a deterministic substep loop (n = clamp(llround(seconds/0.1)); equal dt): snapshot (SIM-010) → waypoint steer with exact-landing (no overshoot) → **RVO velocity-obstacle avoidance** (double cone in relative-velocity space; overlap push with deterministic anti-symmetric fallback for coincident centres; degenerate-zone slide for `r <= dist < R`, landing-exempt — see second review round) → integrate → arc-length expand + bisect containment clamp (SIM-012) → post-move transitions (arrival radius, **landing-only** waypoint advance, route-exhausted reach). Avoidance lives inside this module rather than a separate module — no speculative extension layer. |

## Work packages & plan

All 20 slices S1.1–S7.3 in `implementation-plan.md` are complete (`[x]`),
each with goal, dependencies, files, verification, and status. Explicit
revisions (all in the revision log, none silent):

- `contains` grid index → brute-force scan (S3.3).
- Classical funnel bookkeeping → iterative string-pulling with a
  verification gate (S4.2/4.3); test-found bug in the capsule half-plane
  sign, fixed and regression-tested.
- Internal PIMPL header renamed `simulation.hpp` → `simulation_impl.hpp`
  (S7.3) to kill a same-name collision with the public header and the
  include-ordering hack it forced.
- S7.3 simplify-check worker pass: dead code removal, dedup
  (`err_code`, `strictly_inside`, `live_agent` const overload,
  `reset_motion`, `kArrivalSentinel`, single `landing` array),
  anonymous-namespace moves, and the S63 soak's re-implemented substep
  scheme replaced by an explicit 132-call sequence. Kept deliberately: the
  `kAvoidWeight` blend form (zero-sign behaviour) and the ~1-ulp safety
  clamp.

## Review defect fixes (post-S7.3)

An independent review of the finished library found two reproducible
defects. Both are fixed in `src/vwmini/simulation.cpp` (no public header
changes), each with focused deterministic regressions, and all prior tests
were retained and re-run.

1. **Reflex-corner stall (single agent, `Moving` forever).** A single agent
   on a valid route through the inner corner of an L-shaped mesh got clamped
   at the corner and stalled while still reporting `Moving`, depending on
   the public `step(seconds)` duration (a normal 30 Hz caller hit it).
   Root cause: the route-index advance in the post-move transition phase
   fired on mere proximity — `dist(position, waypoint) <=
   max(1e-4, max_speed*dt)` — instead of on an exact landing. A substep
   whose approach remainder fell in `(ms*dt, 2*ms*dt)` left the agent just
   short of the corner waypoint (or parked in the MSH-006 boundary band
   just past it) and still within the advance threshold, so the waypoint
   was consumed even though the segment from that position to the next
   waypoint cuts non-walkable space. The agent then steered straight into
   the wall, the containment bisection clamped it to ~zero motion, and it
   could never re-target the corner (already consumed) — permanent stall.
   Fix: the route index now advances **only on an exact landing** (the
   substep in which the position is set exactly onto the waypoint, which
   is always within `max_speed*dt` of a consumed waypoint's successor's
   start, and from an exact waypoint the path-validated next segment is
   walkable). The agent at a clamped band point re-targets the corner
   (still within `max_speed*dt`), lands on it next substep, and advances
   then — duration-independent, no new API, no mesh-internal access.
   Regression: `S64_ReflexCorner` — L-shaped mesh, single agent, the exact
   repro scenario at 30 Hz (1 substep/call: n = round((1/30)/0.1) = 1)
   and 10 Hz (1 substep/call),
   checking finiteness, containment and the speed bound after **every**
   step, and asserting `Reached` at the goal.
2. **Overlapping-disc non-separation.** Two agents starting with
   overlapping centres in open space stayed overlapped or were driven
   together, and the pair could converge to permanent contact still
   reporting `Moving`. Three compounding causes in the overlap response:
   (a) **sign error** — the response added `u * magnitude` with `u` the
   unit vector from self to the other agent, i.e. it pushed self *toward*
   the other agent (attraction); the push is now `v - u * magnitude`
   (repulsion). The coincident-centre fallback (deterministic anti-
   symmetric `fallback_dir`) was already correct and is unchanged.
   (b) **insufficient boost** — `kAvoidSeparationBoost` 0.7 leaves a stable
   equilibrium at `d = 0.7r` for head-on full-speed pairs; even 1.0 only
   cancels the worst-case mutual approach (`ms_a + ms_b`), leaving
   `S = (ms_a + ms_b)(r-d)/r → 0` at contact (asymptotic permanent
   contact). Set to **2.0**: `S >= ms_a + ms_b > 0` for every overlap
   depth and any speed asymmetry, so the pair separates in finite time.
   (c) **degenerate-VO re-overlap** — inside the clearance radius but not
   yet overlapping (`r <= dist < R`), the RVO cone's half-angle is >= 90°
   (`sin_h` clamps to 1, `w_boundary == 0`) and the old code "kept"
   the velocity, so a head-on pair that had just separated back into the
   band steered straight into re-overlap (observed: d 0.6 → 0.4). Added a
   direct centre-line repulsion (`v - u * 2*max_speed`) for that band —
   2·`max_speed` per agent guarantees `S >= ms_a + ms_b > 0` for any
   speed combination, so the pair leaves the band with guaranteed progress
   and the well-defined cone (or a lateral pass) takes over. The RVO cone
   branch itself (apex, half-angle, projection, deterministic `perp_dir`
   tie-break) was audited and is correct for `dist >= R`; the relative-
   position convention (`rel = other - self`, `w = v_self - v_other`,
   `v_new = v_other + w_new`) is consistent throughout. (The centre-line
   repulsion added here was itself replaced by the degenerate-zone slide
   in the second review round — see below.)
   Regression: `S62_Avoidance.OverlappingStartIsSeparated` rewritten with
   real assertions (the old version had hidden the sign bug by disabling
   the separation invariant): strict separation growth at every step while
   overlapping, recovery to a clear distance (`d >= r_sum + 0.2`),
   maintained separation (`check_separation` enabled from recovery
   onward), both agents `Reached`, material final separation — plus a
   head-on overlapping variant (the exact "driven together" failure mode).

Verification after the fixes: clean from-scratch builds, **0
warnings/errors under `-Wall -Wextra -Wpedantic` on g++ 16.1 and clang++
22.1, 74/74 tests passed on both**, and an ASan+UBSan build
(`-fsanitize=address,undefined -fno-sanitize-recover=all`) with 74/74
passed. All pre-fix tests retained (the two S62 reconstructions preserve
the original scenario types: same-lane overtake, angled crossing with
speed mismatch).

## Second review round

A second independent review found three further issues. All are fixed in
`src/vwmini/simulation.cpp` and `tests/simulation_test.cpp` (no public
header changes), each with focused deterministic regressions; all prior
tests retained and re-run. Final state: **76 tests**.

1. **Large finite `step()` durations overflowed to NaN (SIM-008/SIM-012).**
   Two independent causes:
   (a) the substep count `n = llround(seconds / 0.1)` is undefined
   behaviour for `seconds` beyond ~9.2e17 (the result exceeds
   `LLONG_MAX`), and
   (b) the 1,000,000-substep cap leaves a huge finite `dt` (e.g. for
   `step(FLT_MAX)` the capped `dt` is ~3.4e32), so a speed-bounded
   displacement `decided * dt` can be finite-but-huge or infinite; the old
   containment clamp bisected the parameter `t in [0, 1]` of the raw
   displacement — with an infinite delta that degenerated to
   `from + delta * 0 = NaN`, and even a finite huge delta needs ~120
   bisections to resolve an O(1)-extent mesh boundary (the agent stayed
   outside). NaN then persisted for the rest of the run.
   Fix: (a) the cap comparison runs **before** `llround` (`scaled >=
   kMaxSubsteps` → cap; otherwise `llround`), so no UB for any finite
   input; large finite durations are still accepted (clamped to the cap,
   per SIM-008's "may be internally subdivided"). (b) the clamp now works
   in **arc length** along the motion direction: exponential expansion to
   a definitely-outside bound (capped at `|delta|` when finite, so the
   old "furthest contained point on the segment" semantics, including the
   re-entry case, are preserved), then bisection on that interval.
   `t = 0` is contained (the previous position was), so the result is
   always finite and contained. Normal (finite, in-mesh) substeps take
   the unchanged fast path and stay bit-identical.
   Regression: `S65_LargeDuration.HugeFiniteStepStaysFiniteAndContained`
   — overlapping-start pair, `max_speed = 1e7`, a single
   `step(FLT_MAX)` call, asserting success, finiteness, containment and
   separation for both agents.
2. **Close-following pair stalled at a reflex corner (SIM-011).** A
   second agent 0.6 m behind a leader rounding the L-mesh inner corner
   deadlocked the pair: neither ever landed the corner waypoint. Root
   cause: in the degenerate zone (`r <= dist < R`, clearance radius
   `R = r_a + r_b + kAvoidMargin = 0.9` m for two default agents) the RVO
   cone half-angle `asin(R / dist)` reaches 90°; its boundary projection
   is then ill-conditioned — the output direction is dominated by a
   near-zero perpendicular component whose sign flips with sub-millimetre
   position changes. The pair flickered between the giant near-
   perpendicular projection and the first-round centre-line repulsion in a
   stable limit cycle: the follower was kicked alternately into each arm
   wall, the leader was deflected into the opposite wall, the containment
   clamp pinned both, and the corner waypoint was never landed.
   Fix: in the degenerate zone the pair **slides** around instead of using
   the projection: the approaching component of the velocity is dropped
   (a receding agent is left alone — the approaching partner's slide
   already makes the centre-line distance grow), the lateral component is
   kept, and the approaching agent gains a small outward drift
   (`kAvoidSlideDrift = 0.25` of `max_speed`). While both agents of a
   conflicting pair are in the zone the centre-line distance is strictly
   increasing (>= 2·drift·`max_speed` head-on, >= drift·`max_speed`
   one-sided), so the pair can never re-enter overlap from the zone and
   leaves it in bounded time; the leader (receding) advances cleanly and
   clears the corner, after which the follower follows. The slide
   **exempts a landing agent** (within `max_speed*dt` of its own target
   waypoint): it glides to the stop uncorrected — its velocity is the
   small `to/dt` gliding speed, it stops at the target, and the overlap
   push (`dist < r`) still guards physical contact. Without the exemption
   an agent whose goal lies inside the zone of a parked agent could never
   land it (same-lane overtaking with the slower agent's goal 0.8 m from
   the faster agent's parking spot regressed before the exemption was
   added). The first-round centre-line repulsion is removed (it was part
   of the flicker cycle).
   Regression: `S64_ReflexCorner.CloseFollowingPairBothReach` — L-mesh,
   leader (2.5, 0.5) → (0.5, 3.0), follower 0.6 m behind (3.1, 0.5) →
   (0.5, 1.5), 600 calls of 1/60 s, per-step invariants (finite,
   contained, speed-capped, discs non-overlapping), both agents `Reached`
   at their exact goals. The follower's goal is 1.5 m from the leader's
   (>= r_a + r_b): the 1 m arm cannot admit a zero-clearance pass of two
   0.25 m discs, so this is the feasible non-passing close-following
   case.
3. **Regression & documentation integrity.** The reconstructed
   `S62_Avoidance.OvertakingPairNeverOverlapsAndBothReach` (same-lane,
   faster agent behind) and the angled-crossing test are present and
   passing with their separation assertions intact (the overtaking test
   directly exercises the landing exemption above: the trailing agent's
   goal is 0.8 m from the leader's parking spot, inside the degenerate
   zone). The wrong "3 substeps/call at 30 Hz" comments were corrected in
   the tests and this report (the subdivision scheme uses **one** substep
   of 1/30 s per call at 30 Hz: n = round((1/30)/0.1) = 1).

## Files changed

- `src/vwmini/`: `geometry.cpp`, `geometry_priv.{hpp,cpp}`,
  `triangulate.{hpp,cpp}`, `mesh_priv.{hpp,cpp}`, `nav_mesh.cpp`,
  `path.{hpp,cpp}`, `simulation.cpp`, `simulation_impl.hpp`
- `tests/`: `CMakeLists.txt`, `test_util.hpp`, `geometry_test.cpp`,
  `triangulate_test.cpp`, `nav_mesh_test.cpp`, `path_test.cpp`,
  `simulation_test.cpp` (76 tests after the review rounds: +2
  `S64_ReflexCorner` single-agent, +1 `S64_ReflexCorner.CloseFollowingPairBothReach`,
  +1 `S65_LargeDuration`, rewritten `S62 OverlappingStartIsSeparated`)
- `CMakeLists.txt`: library sources, `vwmini::vwmini` alias, C++23,
  `-Wall -Wextra -Wpedantic`, system GTest with FetchContent fallback,
  CTest
- `docs/architecture/`: `architecture.md`, `implementation-plan.md` (plus
  `wp5-brief.md`/`wp6-brief.md` working briefs, marked superseded)
- `include/vwmini/*`: **untouched** (verified: mtimes and all
  signatures/defaults/enum values match `public-api.md`)

## Build & test commands (exact, from scratch)

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build --parallel
ctest --test-dir build --output-on-failure
# repeat with -DCMAKE_CXX_COMPILER=clang++ into build-clang/
```

Outcomes: **0 warnings/errors under `-Wall -Wextra -Wpedantic` on g++ 16.1
and clang++ 22.1; 76/76 tests passed on both** (from scratch, after the
second review round), and an ASan+UBSan build (`-fsanitize=address,undefined
-fno-sanitize-recover=all`) with 76/76 passed. No `.clang-format` was
supplied, so no formatting tool was added (NFR-007 requires consistent
formatting, not clang-format); the codebase follows a single
4-space/80-col style. No console output, global mutable state, or
third-party runtime dependencies in library code.

## Deliberate behavior where the contract allows a choice

- **Tie-breaking** (MSH-008/SIM-004): Dijkstra ties broken by
  (distance, cell index); equal-cost route winners are not public behavior —
  the contract says so.
- **Substep scheme**: fixed 10 Hz base with equal `dt` per call and a
  1,000,000-substep cap — "may be internally subdivided" (SIM-008) gives
  latitude; equal `dt` keeps the math bit-deterministic.
- **Avoidance algorithm**: RVO velocity-obstacle projection (SIM-012 says
  any local deterministic method is fine), with tuned, documented
  constants (`kAvoidLookahead=1.5 m`, `kAvoidMargin=0.4 m`,
  `kAvoidWeight=1.0`, `kAvoidSeparationBoost=2.0`,
  `kAvoidSlideDrift=0.25` — see the review-fix sections). SIM-011
  acceptance is exercised by
  deterministic tests checking separation ≥ rᵢ+rⱼ−1e-3 m, finiteness,
  containment, and speed cap at every returned step, with both agents
  reaching their goals; a 500-substep two-run soak asserts bit-identical
  `AgentState` traces.
- **Arrival snapping**: an agent entering its arrival radius (or
  exhausting its route) snaps exactly to the goal, so `Reached` states
  expose the exact goal position — a stronger, cleaner terminal state than
  the contract minimally requires.
- **Error messages**: stable, human-readable strings (only the `ErrorCode`
  is contractual).
- **One-declaration private headers** (`path.hpp`, `triangulate.hpp`) kept
  as module seams rather than inlined — flagged by review, kept by
  decision.
