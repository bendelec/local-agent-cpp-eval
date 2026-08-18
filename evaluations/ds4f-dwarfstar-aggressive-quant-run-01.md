# Evaluation — ds4f-dwarfstar-aggressive-quant-run-01

## Run identity

| Field | Value |
|---|---|
| Model | DeepSeek V4 Flash (DS4F) |
| Runtime | Local Dwarfstar DS4 |
| Precision | Aggressive quantization; exact scheme is externally retained and pending session-metadata import |
| Source snapshot | [`../solutions/ds4f-dwarfstar-aggressive-quant-run-01/`](../solutions/ds4f-dwarfstar-aggressive-quant-run-01/) |
| Source revision | No Git history was captured in the model workspace. Archive tree fingerprint: `38d99164cb39fee59324cb2bdf8b465532e35ac61b624653c8e476d45493338f` (`find`/sorted per-file SHA-256 manifest, then SHA-256). |
| Session archive | [`../sessions/ds4f-dwarfstar-aggressive-quant-run-01/`](../sessions/ds4f-dwarfstar-aggressive-quant-run-01/) |
| Task revision | Pre-planning revision: NFR-009 required architecture documentation; NFR-010 implementation-plan requirement was added after this run began. |
| Evaluation state | **Fixes requested; architecture/quality score deferred. Provenance metadata is partial pending session import.** |

Do not penalize this run for the absent NFR-010 plan artifact; it was not in its task
package. Evaluate its architecture document and source against the contemporaneous
requirements after functional repairs are submitted.

## Intake and gates

| Check | Result | Evidence |
|---|---|---|
| Fresh CMake configure/build | Pass | GCC 16.1.1; library, native tests, and conformance executables built warning-clean under supplied warning flags. |
| Public API compatibility | Pass | Conformance suite compiled unchanged against `vwmini::vwmini`. |
| Native tests | Pass | `geometry_test`, `navmesh_test`, and `sim_test`: 3/3 passed. |
| Core safety gate | Pass in exercised tracks | No crash, NaN, or observed out-of-mesh result in conformance. Further source-review safety findings are listed below. |

## Conformance results

| Track | Result | Evidence |
|---|---:|---|
| G — Geometry | Pass (15/15 tests) | Initial collinear-output count failure was an evaluator-test defect and was corrected; all geometry tests then passed. |
| N — Navmesh and paths | 22/24 tests pass | Direct boundary-tolerance route was not exactly `[start, goal]`; L-shaped bent route included an out-of-mesh segment. |
| S — Agent lifecycle and stepping | Pass (28/28 tests) | All exposed lifecycle/stepping checks passed. |
| C — Local crowd behavior | 1/2 tests pass | Overtaking passed. Ordinary crossing left both agents `Moving` after fixture duration rather than `Reached`. |

Command:

```sh
./evaluator/conformance/run.sh \
  solutions/ds4f-dwarfstar-aggressive-quant-run-01 \
  /tmp/vwmini-ds4f-dwarfstar-aggressive-quant-run-01
```

The runner was corrected during this evaluation so a failed track no longer prevents
independent later tracks from running.

## Required fixes

The archived solution is unchanged. Apply repairs only in a new resubmission/workspace,
then rerun the complete conformance suite.

### PTH-001 — Make route containment one correct shared predicate

**Evidence:** `FindPath.BoundaryToleranceDirectPathPreservesExactEndpoints` returned
five points, not exactly `[start, goal]`; `FindPath.BentPathShortensAroundReflexCorner`
returned a path with an out-of-mesh segment.

**Cause:** `src/nav_mesh.cpp` decides segment containment/visibility solely by absence
of a *proper* boundary-edge intersection. That neither implements MSH-006's epsilon
boundary policy nor proves an entire segment is covered by the union. The fallback
corridor path also appends its goal twice after reconstructing a chain that already
contains the goal.

**Required repair:** Implement one epsilon-aware `segment_contained(mesh, a, b)` that
proves coverage of all segment parameters by the triangle union. Use it for direct-path
selection, corridor/visibility edges, and final path validation. Preserve exact caller
endpoints and do not append the goal a second time. Add tests for the two failing cases
and assertions over **segments**, not only returned points.

### CRO-001 — Make ordinary crossing progress without sacrificing safety

**Evidence:** `Crowd_Crossing.TwoAgentsCrossAndReachGoals` left both agents `Moving`
after the scenario duration. Overtaking passed.

**Cause:** Each avoidance decision compares its candidate with every other agent's old
snapshot velocity, although all selected velocities are subsequently applied together.
A strict 0.5-second horizon/margin can reject all useful forward/sideways choices; the
only fallback is zero velocity, yielding a stall.

**Required repair:** Select deterministic pairwise-compatible steering velocities, or
use a clear stable priority/reservation policy and validate the actual simultaneous
choices. Rank feasible choices by progress and retain a safe yielding option before
stopping. Add the fixture-duration `Reached` assertions to the native tests.

### SIM-001 — Validate the displacement actually committed

**Source-review evidence:** `applyMove` may snap to a waypoint whenever the selected
displacement length exceeds remaining waypoint distance, even if its direction is
lateral. That snap was not the collision-checked candidate. Endpoint-only mesh checking
also does not prove a motion segment stays inside a concave mesh.

**Required repair:** Snap only when movement reaches the waypoint along the selected
ray; otherwise integrate the chosen displacement. Validate the actual swept centre
segment with the same containment predicate before committing it.

### MSH-001 — Reject coincident triangle interiors

**Source-review evidence:** the overlap test checks only strict vertex inclusion and
proper edge crossings. Identical CCW triangles can therefore be accepted despite fully
overlapping interiors.

**Required repair:** Handle coincident/collinear edges or explicitly reject equal
triangles before general overlap testing. Add an identical-triangle regression test.

## Additional repair recommendations

- Complete outline self-intersection handling: reject non-adjacent endpoint touches and
  collinear edge overlaps; apply the specified degeneracy threshold and validate final
  output triangles.
- Do not use mutable distance/score arrays through `priority_queue` comparators; store
  immutable queue entries and discard stale entries when popped.
- Handle initial disc overlap with a deterministic separation attempt instead of an
  immediate zero-velocity fallback.
- Avoid converting huge finite `step` durations through an out-of-range `float` to
  `int`; use a safe remaining-time/substep scheme.
- Use `double` intermediates for geometry predicates/distances to avoid finite extreme
  coordinates overflowing to non-finite values.

## Architecture and quality review

Deferred until required functional repairs pass. The submitted architecture document is
present and appears to describe the intended modules; it will be reviewed against the
corrected code rather than scored independently of material behavior defects.

## Final result

```text
Build/API gate: PASS
Safety gate:    provisionally PASS in exercised tests; source-review repairs pending
G pass, N fail (2 tests), S pass, C fail (1 test)
Tests / Architecture / C++ quality: deferred pending repair
Total: not assigned
```

## Repair lineage

The separately archived first repair submission is evaluated in
[`ds4f-dwarfstar-aggressive-quant-run-01-repair-01.md`](ds4f-dwarfstar-aggressive-quant-run-01-repair-01.md).
