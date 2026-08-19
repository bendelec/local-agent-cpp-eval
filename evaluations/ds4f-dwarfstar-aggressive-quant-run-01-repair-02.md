# Final Evaluation — ds4f-dwarfstar-aggressive-quant-run-01-repair-02

## Identity and scope

| Field | Value |
|---|---|
| Run | DeepSeek V4 Flash, local Dwarfstar DS4, aggressive quantization |
| Submission | Second and final permitted repair of [`ds4f-dwarfstar-aggressive-quant-run-01`](ds4f-dwarfstar-aggressive-quant-run-01.md) |
| Source snapshot | [`../solutions/ds4f-dwarfstar-aggressive-quant-run-01-repair-02/`](../solutions/ds4f-dwarfstar-aggressive-quant-run-01-repair-02/) |
| Source revision | No Git history captured in the model workspace. Archive tree fingerprint: `3e57314a87fc2dfd0ea73686ffffc9e763c1beb1f2f287e079ceb3b9c8be689e` (inside archive: `find . -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum`). |
| Provenance | Same externally retained session as the original run; exact runtime/session export is pending import. |
| Task revision | Pre-NFR-010 planning revision; no implementation-plan penalty applies. |
| Repair limit | Reached: this evaluation is final. |

The original and first-repair archives remain immutable. This record evaluates only the
final repair snapshot, while preserving the full repair lineage.

## Build and test evidence

| Check | Result |
|---|---|
| Fresh CMake/configure/build via conformance runner | Pass; public API compatibility preserved |
| Candidate-native CTest | 3/3 pass |
| Candidate clang-format dry run | Pass |
| Public conformance | **69/69 pass**: G 15/15, N 24/24, S 28/28, C 2/2 |
| Sanitized valid-input probe | **Fail**: UBSan reports an out-of-range float-to-`int` conversion in `Simulation::step(FLT_MAX)` |

Commands included:

```sh
./evaluator/conformance/run.sh \
  solutions/ds4f-dwarfstar-aggressive-quant-run-01-repair-02 \
  /tmp/vwmini-ds4f-dwarfstar-aggressive-quant-run-01-repair-02

ctest --test-dir /tmp/vwmini-ds4f-dwarfstar-aggressive-quant-run-01-repair-02/candidate_under_test \
  --output-on-failure
```

The sanitizer failure occurs at `src/simulation.cpp:501`: a finite, positive `FLT_MAX`
duration is accepted by the documented interface, then `ceil(seconds / 0.1f)` is converted
to `int` out of range. This is undefined behavior on valid public input.

## Score

| Area | Score | Evidence |
|---|---:|---|
| Geometry and mesh validation | 12 / 20 | Baseline geometry/mesh conformance passes, and the repair added useful self-touching/overlap tests. However, a positive-area degenerate outline (`{(0,0),(1,0),(0.5,1e-9)}`) is accepted, large-coordinate vertex-only mesh contact can be rejected, and finite norm behavior remains false for sufficiently large values. |
| Pathfinding | 11 / 20 | All supplied direct, disconnected, and bent-route tests pass; the duplicate final goal is fixed. The claimed continuous coverage still merges real uncovered parameter gaps and can return a direct route through an epsilon-scale physical gap. |
| Agent lifecycle and stepping | 13 / 20 | Supplied lifecycle/stepping tests pass, but valid huge finite duration invokes UB. IDs also wrap and can eventually reuse an identifier that contract semantics require to stay invalid after removal. |
| Local crowd behavior | 12 / 15 | Supplied crossing and overtaking both pass; the final repair adds committed-segment checking. The resolution loop/documentation overstates its guarantee, and swept mesh containment remains capped sampling for unbounded `max_speed`. |
| Tests and functional discipline | 7 / 10 | Fast deterministic CTest coverage and several meaningful repair regressions. Missing tests allowed all final defects above, including extreme finite inputs and continuous epsilon-scale cases. |
| Architecture | 6 / 10 | Clear three-module split, ownership, immutable mesh sharing, and useful architecture document. However, the document claims exact/no-cap containment and stronger avoidance/numeric guarantees than the implementation establishes; repair complexity is concentrated in large coupled geometry/avoidance helpers. |
| C++ quality | 3 / 5 | RAII, value semantics, formatting, and warning-clean normal build are good. Undefined behavior and unsafe/extreme numerical operations materially reduce robustness. |
| **Raw total** | **64 / 100** | — |

### Gate and final score

```text
Build/API gate: PASS
Safety gate:    FAIL — UBSan-confirmed undefined behavior for a valid finite step duration
Conformance:    G 15/15, N 24/24, S 28/28, C 2/2
Raw total:      64/100
Hard cap:       40/100
Final score:    40/100
```

The rubric caps a submission at 40 when a core safety condition fails. The cap applies:
`step(FLT_MAX)` is finite and non-negative, therefore valid under SIM-008, but produces
undefined behavior rather than a safe result.

## Material unmet requirements and defects

- **SIM-008 / NFR-006:** `Simulation::step(FLT_MAX)` performs an out-of-range conversion
  to `int`; UBSan confirms the undefined behavior.
- **SIM-002 / SIM-003:** `continuousContained` joins coverage intervals separated by a
  non-zero parameter tolerance. A real, non-contained physical gap can be accepted as a
  direct path. Swept simulation membership also retains capped finite sampling despite
  unbounded public `max_speed`.
- **MSH-004:** ordinary large-coordinate triangles that only meet at one vertex can be
  rejected as overlapping due to rounded normalized overlap projection.
- **MSH-001:** `length({3e38f, 3e38f})` returns infinity despite the public-header
  finite-input guarantee; `normalized` then returns zero for a non-zero vector.
- **MSH-002 / MSH-003:** a finite positive-area but sub-threshold-degenerate outline is
  accepted and emits a degenerate triangle.
- **SIM-013:** ID wraparound permits eventual reuse of an old removed identifier.

## Strengths

- The final repair demonstrably fixed the prior public conformance failures, including
  direct boundary paths, L-shaped paths, ordinary supplied crossing, and endpoint-order
  handling.
- Public API, CMake target, immutability/value ownership approach, and standard lifecycle
  behavior are preserved.
- The source is generally readable, warning-clean under normal flags, formatted, and has
  a coherent module split.
- The model responded to feedback with targeted native regressions and updated
  architecture/completion documentation.

## Documentation assessment

The final architecture document is complete in structure and describes actual component
boundaries, ownership, and data flow. It is not fully reliable as an implementation
description: its claims of exact continuous containment, no effective sampling cap,
robust finite numeric behavior, and unconditional committed-segment separation are
stronger than the final source establishes. The completion report likewise overstates
those repairs. These are scoring evidence, not modifications to the immutable submission.

## Final disposition

No further repair attempt will be requested. Preserve this archive and report alongside
its original and first-repair predecessors as the final result for this model run.
