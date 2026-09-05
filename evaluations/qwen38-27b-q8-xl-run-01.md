# Evaluation — Qwen 3.8 27B

## Run identity

| Field | Value |
|---|---|
| Model | Qwen 3.8 27B |
| Runtime | Local Lemonade / llama.cpp; `Qwen3.8-27B-UD-Q8_K_XL` |
| Task revision | Current VWmini task, including NFR-009 architecture and NFR-010 implementation-plan deliverables |
| Final source | [`../solutions/qwen38-27b-q8-xl-run-01-repair-02/`](../solutions/qwen38-27b-q8-xl-run-01-repair-02/) |
| Final tree fingerprint | `006afde4c37dfd50a7e5df350c06cd5806ebc7f6eb303aa7793bce84f9276b10` |
| Repair limit | Two repair prompts |
| Final result | **68 / 100** |

## Initial state

The one-shot submission was unusually strong. It built cleanly, preserved the public API,
passed its native suite (72/72), and had a complete C++23 architecture with real design and
implementation-plan documentation. Geometry, mesh construction, and pathfinding were the
strongest parts.

Against the then-current 72-test public suite it scored **69/72**: geometry 15/15,
navmesh/path 24/24, simulation 28/29, and crowd 2/4. The missed behaviors were a
single-agent reflex-corner stall, no material recovery from an initially overlapping pair,
and close-following progress through a reflex corner.

## Final state

The final source builds warning-clean with GCC and Clang. Its native suite passes **76/76**
under GCC and under ASan/UBSan. At finalization it passed **71/72** then-current conformance
tests. Conformance revision 2 added five navmesh/path cases and reran the archived final
snapshot: it now passes **75/77** (geometry 15/15, navmesh/path 28/29, lifecycle 29/29,
crowd 3/4). It fails finite-extreme-coordinate mesh containment; the close-following
reflex-corner test also fails because the follower remains `Moving`.

Visual testing confirms that a nearby same-direction follower can still cause a leading
agent to become stuck at a valid L-corner. This is a practical local-avoidance progress
failure, not an infeasible shared-goal case.

The final repair also has three material findings not exposed by its native sanitizer suite:

- A finite extreme-coordinate triangle is accepted but its strictly interior point is not
  contained. This is now the failing revision-2 normative navmesh/path test and demonstrates
  overflow in mesh predicates.
- Valid `max_speed == FLT_MAX` together with a huge finite step can overflow the float
  avoidance push and propagate NaN state.
- The landing exemption used to improve degenerate-zone progress can permit a fast moving
  disc's swept segment to pass through a stationary disc when its endpoints are clear.

These are substantial functional deductions, but they are not an observed memory-safety or
undefined-behavior sanitizer failure. The `max_speed == FLT_MAX` case is a targeted
source-review probe, not a failing normative-conformance fixture; it therefore does not
invoke the hard safety cap.

Reproduction used:

```sh
./evaluator/conformance/run.sh solutions/qwen38-27b-q8-xl-run-01-repair-02 /tmp/qwen-final
cmake -S solutions/qwen38-27b-q8-xl-run-01-repair-02 -B /tmp/qwen-native -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/qwen-native --parallel && ctest --test-dir /tmp/qwen-native --output-on-failure
```

## Implementation size and complexity

Source-only non-comment LOC is **1,292 NCLOC**, counted as every nonblank `src/` line
remaining after lexical removal of line and block comments. This excludes public headers,
tests, documentation, and generated files. The largest implementation unit is
`src/vwmini/path.cpp` at 398 NCLOC, followed by `simulation.cpp` at 394.

Clang's CFG dump gives a maximum source-function McCabe complexity of **40** for
`Simulation::step(float)` (90 basic blocks, 128 edges; `E - N + 2`). Its maximum lexical
brace nesting is **6** levels. These are compiler/configuration-dependent diagnostics,
not correctness scores, but they provide useful context for the avoidance review.

## Score

| Area | Score | Assessment |
|---|---:|---|
| Architecture and dependency design | 14 / 20 | Public ownership boundaries are sound, but the avoidance subsystem remains under-decomposed and storage-coupled. |
| Decomposition and complexity | 8 / 20 | `Simulation::step` is a 207-line multi-phase orchestrator (CFG 40, nesting 6); pathfinding also combines several bespoke numerical algorithms in one unit. |
| C++ clarity and discipline | 7 / 10 | RAII, values, `std::expected`, and comments are strong, but parallel arrays and phase-coupled float avoidance reduce local clarity. |
| Tests and functional discipline | 11 / 15 | 76 focused tests and sanitizer runs are useful, but the easier corner fixture and missing independent extreme/swept-contact checks leave important gaps. |
| Functional conformance beyond the gate | 28 / 35 | Revision-2 conformance is 75/77; finite-extreme mesh containment, the remaining corner deadlock, extreme-speed NaN, and tunnelling probes are material. |
| **Total** | **68 / 100** | |

## Repair prompts and intermediate result

Two repair prompts were used.

1. **First repair:** fixed the single-agent reflex waypoint-consumption stall and the
   overlap-response sign/progress defect. It was a strong, targeted improvement, but did
   not cover huge finite durations or close-following corner behavior.
2. **Second repair:** targeted huge finite-duration safety, close-following reflex-corner
   progress, restoration of avoidance regressions, and documentation accuracy. It added
   detailed regressions and fixed several issues, but its close-following scenario used a
   less demanding terminal arrangement than the public fixture. The remaining public
   corner failure and review findings above prevent a higher score.

## Final assessment

Qwen produced a substantially stronger implementation than the other evaluated model: the
core geometry/path system is reliable, the architecture is coherent, and repair prompts led
to real improvements. Its main weakness is avoidance complexity: successive local patches
addressed individual failures but did not yield a robust, general corner-following policy.
The result is a credible small navigation library with meaningful crowd limitations, rather
than release-quality local avoidance.
