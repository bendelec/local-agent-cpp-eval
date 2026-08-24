# Evaluation — DeepSeek V4 Flash

## Run identity

| Field | Value |
|---|---|
| Model | DeepSeek V4 Flash |
| Runtime | Local Dwarfstar DS4; aggressive quantization |
| Task revision | Pre-planning revision: NFR-009 only; no NFR-010 implementation-plan penalty applies |
| Final source | [`../solutions/ds4f-dwarfstar-aggressive-quant-run-01-repair-02/`](../solutions/ds4f-dwarfstar-aggressive-quant-run-01-repair-02/) |
| Final tree fingerprint | `3e57314a87fc2dfd0ea73686ffffc9e763c1beb1f2f287e079ceb3b9c8be689e` |
| Repair limit | Two repair prompts |
| Final result | **40 / 100 capped** (raw 62 / 100) |

## Initial state

The initial submission built, preserved the public API, and passed the geometry and lifecycle
tracks. It failed two navmesh/path checks and ordinary crossing progress. Its native tests
were small (3/3), and architecture/quality scoring was deferred while functional repairs
were requested.

## Final state

The final source builds and exposes the required API. It passes **71/72** current public
conformance tests: geometry 15/15, navmesh/path 24/24, simulation 29/29, and crowd 3/4.
It passes the close-following reflex-corner fixture but fails the explicit initially
overlapping-disc recovery fixture. Candidate-native tests pass 3/3 and normal formatting
checks pass.

A valid finite `step(FLT_MAX)` triggers an out-of-range float-to-`int` conversion in
`Simulation::step`; UBSan reports it. This is undefined behavior on valid public input and
triggers the safety cap.

Reproduction used:

```sh
./evaluator/conformance/run.sh solutions/ds4f-dwarfstar-aggressive-quant-run-01-repair-02 /tmp/ds-final
ctest --test-dir /tmp/ds-final/candidate_under_test --output-on-failure
```

## Score

| Area | Score | Assessment |
|---|---:|---|
| Geometry and mesh validation | 12 / 20 | Baseline checks pass, but finite extreme norms, degeneracy, and large-coordinate topology remain fragile. |
| Pathfinding | 11 / 20 | Main route fixtures pass, but tolerance-based interval joining can accept a physical uncovered gap. |
| Agent lifecycle and stepping | 13 / 20 | Normal lifecycle passes; valid huge duration invokes undefined behavior and identifiers can wrap. |
| Local crowd behavior | 10 / 15 | Crossing, overtaking, and close following pass; simple open-space initial overlap does not recover. |
| Tests and functional discipline | 7 / 10 | Deterministic tests and repair regressions exist, but missed all final numerical and overlap defects. |
| Architecture | 6 / 10 | Sensible module split and ownership, but documentation overstates containment and avoidance guarantees; complex helpers are tightly coupled. |
| C++ quality | 3 / 5 | Generally readable, RAII/value-oriented, and warning-clean in normal builds; valid-input UB is a material defect. |
| **Raw total** | **62 / 100** | |

## Safety gate

```text
Build/API gate: PASS
Safety gate:    FAIL — UBSan reports undefined behavior for valid step(FLT_MAX)
Final score:    40 / 100 (hard cap)
```

## Repair prompts and intermediate result

Two repair prompts were used.

1. **First repair:** improved boundary-tolerance routing, ordinary crossing, self-touch
   validation, and several mesh/motion cases. It still emitted an invalid L-shaped route
   with a duplicate terminal point and left important numeric/containment weaknesses.
2. **Second repair:** fixed the remaining public route failure and most supplied crowd
   fixtures. It did not fix huge-duration UB, continuous-coverage tolerance gaps, or
   material separation recovery from initial overlap.

## Final assessment

DeepSeek improved substantially under repair, particularly on ordinary path and crowd
fixtures, but its final source remains unsuitable for release because valid input can invoke
undefined behavior. The hard cap is appropriate despite useful architecture and repair work.
