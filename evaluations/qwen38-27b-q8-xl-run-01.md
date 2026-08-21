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
| Public conformance | **69/69 pass**: geometry 15/15, navmesh/path 24/24, simulation 28/28, crowd 2/2 |
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
| Agent lifecycle and stepping | 20 / 20 | All configuration/error/state/lifecycle/arrival/speed/containment checks pass. Agent ownership is clear, operations are transactional where required, and large-duration behavior is exercised by both suites. |
| Local crowd behavior | 14 / 15 | The supplied crossing and overtaking fixtures pass, as do the candidate's crossing, overtaking, angled-crossing, overlap-robustness, non-interference, and bit-identical soak tests. Exploratory two-agent stress found that very closely spaced, near-parallel discs can enter a deep overlap after initially benign motion; the local method is strong for the required normal fixtures but not broadly robust beyond them. |
| Tests and functional discipline | 10 / 10 | 72 focused deterministic native tests cover error paths, geometry, topology, routing, lifecycle, motion, the required interactions, and a deterministic soak. No fixture-coordinate hardcoding or test weakening was found. |
| Architecture | 9 / 10 | The implementation has cohesive geometry, triangulation, mesh, path, and simulation seams; immutable mesh sharing and PIMPL ownership are appropriate. `architecture.md`, its diagram, the explicit revision log, and the completion report closely reflect the source. One point is reserved because the documented RVO description and strong global containment/avoidance wording are more confident than the narrowly scoped local behavior demonstrates. |
| C++ quality | 4 / 5 | C++23, RAII/value semantics, no mutable global state, clear names, const query paths, warning-clean compilation, and sanitizers are all strong. The large, mathematically dense avoidance function is harder to audit and tune than the otherwise direct implementation. |
| **Total** | **97 / 100** | — |

### Gates and final score

```text
Build/API gate: PASS
Safety gate:    PASS — fresh ASan/UBSan suite reports no diagnostic
Conformance:    G 15/15, N 24/24, S 28/28, C 2/2
Final score:    97/100
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

## Scope note on crowd avoidance

The score does not claim universal crowd collision avoidance: SIM-011 intentionally limits
acceptance to feasible two-agent crossing/overtaking cases in open convex space, and those
pass. During extra deterministic random two-agent probing, a pair of radius-0.2 agents
starting about 0.434 m apart on near-parallel, slightly convergent goal paths later
oscillated into overlap. This does not fail the supplied acceptance fixtures, but it is why
local crowd behavior is not scored as universally robust or given its final point.

## Documentation assessment

- `docs/architecture/architecture.md`: current module boundaries, Mermaid dependency
diagram, ownership, numerical policy, containment seam, path gate, and avoidance choices
match the implementation.
- `docs/architecture/implementation-plan.md`: ordered small slices are all marked done,
with expected files and verification; changes are explicitly recorded in its revision log.
- `docs/architecture/completion-report.md`: present in the final snapshot and accurately
summarizes the build, test, and design state.

## Final finding

This submission passes every fixed public conformance track and all of its substantially
broader native tests on the first attempt. The only reservation is the expected limitation
of a tuned local avoidance policy outside the narrowly specified normal-crowd cases; it is
not a build, API, safety, mesh, path, or lifecycle defect.
