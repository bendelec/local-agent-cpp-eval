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
| Final result | **40 / 100 capped** (raw 59 / 100) |

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

## Implementation size and complexity

Source-only non-comment LOC is **1,232 NCLOC**, counted as every nonblank `src/` line
remaining after lexical removal of line and block comments. This excludes public headers,
tests, documentation, and generated files. The largest implementation unit is
`src/nav_mesh.cpp` at 553 NCLOC, followed by `simulation.cpp` at 414.

Clang's CFG dump gives a maximum source-function McCabe complexity of **33** for
`triangulate_simple_polygon` (72 basic blocks, 103 edges; `E - N + 2`). Its maximum
lexical brace nesting is **5** levels. These are compiler/configuration-dependent
diagnostics, not correctness scores, but they give scale to the architecture review.

## Score

| Area | Score | Assessment |
|---|---:|---|
| Architecture and dependency design | 13 / 20 | Public dependency direction and RAII ownership are sound, but tests cross private boundaries and navmesh carries redundant topology representations. |
| Decomposition and complexity | 8 / 20 | CFG 33 and nesting 5 permit a higher ceiling, but duplicated avoidance candidate search and concentrated navmesh/simulation policies materially lower the review score. |
| C++ clarity and discipline | 7 / 10 | Ownership/value semantics are clear, but unused/redundant state and long algorithmic functions make safe changes costly. |
| Tests and functional discipline | 10 / 15 | Deterministic tests and repair regressions exist, but production helpers serve as test oracles and final numerical/overlap defects were missed. |
| Functional conformance beyond the gate | 21 / 35 | 71/72 public conformance is offset by interval-coverage, numeric, and overlap-recovery defects. |
| **Raw total** | **59 / 100** | |

## Safety gate

```text
Build/API gate: PASS
Safety gate:    FAIL — UBSan reports undefined behavior for valid step(FLT_MAX)
Final score:    40 / 100 (hard cap; raw 59)
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
