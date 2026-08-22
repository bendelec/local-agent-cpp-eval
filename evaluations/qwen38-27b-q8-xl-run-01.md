# Final Evaluation — qwen38-27b-q8-xl-run-01

## Identity and scope

| Field | Value |
|---|---|
| Run | Qwen 3.8 27B |
| Runtime | Local Lemonade / llama.cpp; `Qwen3.8-27B-UD-Q8_K_XL` |
| Submission | First complete attempt |
| Source snapshot | [`../solutions/qwen38-27b-q8-xl-run-01/`](../solutions/qwen38-27b-q8-xl-run-01/) |
| Source revision | No Git history captured in the model workspace. Archive tree fingerprint: `7f3b44f7e787e66c3e3362d4bb27ef8909f9827f1d143a74331babad04e3a5bd` (`find . -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum`). |
| Task revision | Current task, including NFR-009 and NFR-010 architecture/planning deliverables. |
| Repair limit | Not applicable; this is the first attempt and final submission. |

The archive was copied only after the model wrote its completion report. Generated
`build*` and `CMakeFiles` output from the live workspace was deliberately excluded; no
generated artifacts are in the immutable source snapshot.

## Build and test evidence

| Check | Result |
|---|---|
| Fresh CMake/configure/build via conformance runner | Pass; public API compatibility preserved |
| Public conformance | **69/72 pass**: geometry 15/15, navmesh/path 24/24, simulation 28/29, crowd 2/4 |
| Candidate-native CTest from fresh snapshot build | **72/72 pass** |
| Candidate-native CTest with ASan + UBSan | **72/72 pass**; no sanitizer diagnostic |
| Independent fresh Clang library build | Pass with `-Wall -Wextra -Wpedantic`; no warnings |

Commands included:

```sh
./evaluator/conformance/run.sh \
  solutions/qwen38-27b-q8-xl-run-01 \
  /tmp/vwmini-qwen38-27b-q8-xl-run-01-final

cmake -S solutions/qwen38-27b-q8-xl-run-01 \
  -B /tmp/vwmini-qwen38-27b-q8-xl-run-01-native \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/vwmini-qwen38-27b-q8-xl-run-01-native --parallel
ctest --test-dir /tmp/vwmini-qwen38-27b-q8-xl-run-01-native --output-on-failure
```

## Score

| Area | Score | Evidence |
|---|---:|---|
| Geometry and mesh validation | 20 / 20 | All geometry and mesh conformance checks pass. The implementation validates finite/simple/CCW outlines, produces deterministic ear-clipped triangles, validates mesh topology, and implements allocation-free edge-tolerance containment. |
| Pathfinding | 20 / 20 | All direct, disconnected, connected, bent-corridor, endpoint-preservation, and determinism checks pass. The implementation uses shared-edge adjacency, deterministic Dijkstra selection, portal string-pulling, and an analytic segment-containment gate. |
| Agent lifecycle and stepping | 15 / 20 | Configuration/error/state/lifecycle basics and straight-line movement pass. However, a single agent on a valid bent L-corridor route can advance from a portal corner prematurely, steer through the missing quadrant, be containment-clamped at the reflex corner, and never reach its valid goal. The failure depends on public caller timestep (fails at 30 Hz, passes at 60 Hz). |
| Local crowd behavior | 9 / 15 | The crossing/overtaking fixtures pass, but the explicit initially-overlapping robustness case fails even in open space. In `avoidance_velocity`, the overlap response adds the direction from self to other, attracting rather than separating the discs; the maximum separation remains 0.1 m in the regression. The candidate's identically named native test disabled separation assertions and did not expose this. |
| Tests and functional discipline | 7 / 10 | 72 focused deterministic native tests and sanitizers are useful, but two visible behavioral defects passed because the motion suite tested only straight-line simulation and the overlap test weakened its central separation assertion. No fixture-coordinate hardcoding was found. |
| Architecture | 7 / 10 | The module split, immutable ownership, diagram, plan, and revision log remain good. However, the documentation calls the response RVO and says overlapping discs are pushed apart, while the implementation uses the attraction sign; the route-state transition is also insufficiently tested at portal corners. |
| C++ quality | 4 / 5 | C++23, RAII/value semantics, no mutable global state, clear names, warning-clean compilation, and sanitizers remain strong. The large, mathematically dense avoidance function obscured a fundamental directional-sign error. |
| **Total** | **82 / 100** | — |

### Gates and final score

```text
Build/API gate: PASS
Safety gate:    PASS — fresh ASan/UBSan suite reports no diagnostic
Public conformance: G 15/15, N 24/24, S 28/29, C 2/4
Final score:    82/100
```

## Strengths

- This is a complete first-attempt implementation, not a minimal conformance patch.
- Public API and target shape are preserved exactly; the library is a small C++23 static
  target with no runtime dependency beyond the standard library.
- Mesh construction and pathfinding have unusually strong defensive checks: topology
  validation, deterministic route selection, and a post-construction containment gate.
- Simulation state is locally owned, snapshot-stepped, deterministic, and independently
  tested under GCC, Clang, ASan, and UBSan.
- The architecture and implementation plan are genuine, current implementation documents
  rather than generic boilerplate. The plan records meaningful design changes, including
  removal of an unjustified spatial index and a private-header collision fix.

## Known motion and avoidance defects

The following deterministic public-API failures are covered by the current conformance suite;
the reference implementation passes each.

1. `Simulation_Step.BentRouteAroundReflexCornerReachesGoal` runs a single agent from
   `{1.9, 0.55}` to `{0.9, 1.75}` through the literal L mesh at 30 Hz. Qwen stalls roughly
   0.76 m from its goal at the reflex corner. The cause is the post-move route loop advancing
   a waypoint when it is merely within `max_speed * dt`, allowing the following segment to cut
   through non-walkable space; the containment clamp then prevents further progress.
2. `Crowd_OverlapRecovery.InitiallyOverlappingDiscsSeparate` uses the SIM-010 edge case of
   two overlapping radius-0.25 agents in open space and requires only material separation
   recovery, not a general crowd solver. Qwen never exceeds its 0.1 m initial separation.
   Its native `OverlappingStartIsSeparated` test disabled overlap assertions and accepted
   unchanged separation, despite its name and comments.
3. `Crowd_ReflexCornerFollowing.CloseAgentsBothRoundCornerAndReach` uses two initially
   non-overlapping agents following in the same direction through an L-shaped reflex corner
   to distinct, collision-feasible goals. Both remain `Moving` rather than reaching; a nearby
   follower should not deadlock the leading agent at a valid corner.

The multiple agents manually assigned exactly the same goal in the lab are not independently
scored: discs cannot all occupy the same terminal point collision-free. That infeasible setup
is distinct from these reproducible defects.

## Documentation assessment

- `docs/architecture/architecture.md`: current module boundaries, Mermaid dependency
diagram, ownership, numerical policy, containment seam, path gate, and avoidance choices
match the implementation.
- `docs/architecture/implementation-plan.md`: ordered small slices are all marked done,
with expected files and verification; changes are explicitly recorded in its revision log.
- `docs/architecture/completion-report.md`: present in the final snapshot and accurately
summarizes the build, test, and design state.

## Final finding

This initial submission has strong geometry, mesh, and path behavior, but the current
conformance suite identifies motion and local-avoidance defects that are addressed by the
subsequent repair process.
