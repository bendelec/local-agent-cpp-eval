# Evaluation — DeepSeek V4 Flash (unquantized)

## Run identity

| Field | Value |
|---|---|
| Model | DeepSeek V4 Flash |
| Runtime | Venice hosted, `deepseek-v4-flash-0731` |
| Task revision | Current VWmini task, including NFR-009 and NFR-010 |
| Final source | [`../solutions/ds4f-unquant-run-01-repair-02/`](../solutions/ds4f-unquant-run-01-repair-02/) |
| Final tree fingerprint | `af255eadc7c4060bc58ba4182fa7bd3b71266dc30c08c837396d52dc5434d012` |
| Repair limit | Two repair prompts |
| Final result | **67 / 100** |

## Initial state

The initial submission built, preserved the fixed public API, and included substantive
architecture and implementation-plan documents. Its native tests passed. Public
conformance was **69/72**: geometry 15/15, lifecycle 29/29, navmesh/path 23/24, and
crowd 2/4. It failed one path case plus ordinary crossing and close following through a
reflex corner.

## Final state

The final source passes **71/72** public conformance tests: geometry 15/15,
navmesh/path 24/24, lifecycle 29/29, and crowd 3/4. Crossing, overtaking, and initial
overlap recovery pass. The close-following reflex-corner fixture still fails: the leader
reaches its goal, while the follower remains `Moving` at approximately `(1.0001, 2.5768)`,
about 1.51 m from its goal, unchanged through 30 simulated seconds.

The candidate-native suite reports 34/34 passing cases under GCC and Clang. Its native
ASan/UBSan build passes. Reproduction used:

```sh
evaluator/conformance/run.sh solutions/ds4f-unquant-run-01-repair-02 /tmp/ds4f-unquant-r2-conformance
cmake -S solutions/ds4f-unquant-run-01-repair-02 -B /tmp/ds4f-unquant-r2-native -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/ds4f-unquant-r2-native --parallel
ctest --test-dir /tmp/ds4f-unquant-r2-native --output-on-failure
```

The second repair replaces simple lane-side steering with a large ORCA-style half-plane
solver and persistent per-pair encounter state. This fixes the public crossing case and
looks materially cleaner in the visual laboratory. Its qualitative weakness is excessive
stand-off: an arriving agent can dither in wide arcs around a cluster of reached agents
several radii away.

Independent review also found—and direct probes reproduced—material defects outside the
current public suite:

- Two initially non-overlapping radius-0.25 same-direction agents at `(1,2)` and
  `(1.55,2)`, with speeds 2 and 0.5 and rightward goals, have only `0.475` centre
  separation after `step(0.05f)`. The pair begins inside the implementation's enlarged
  envelope but has zero prior relative velocity; the cone calculation installs no
  constraint. This violates SIM-011.
- Valid `FLT_MAX` radii overflow pair arithmetic and yield non-finite positions and
  velocities after one valid step. The public agent-validation contract permits finite
  positive radii. This is a valid-input numerical robustness failure, though it is not a
  sanitizer-reported memory or undefined-behavior failure and was not observed in
  normative conformance; it does not invoke the 40-point safety cap.
- The half-plane candidate search omits speed-circle/constraint-line intersections, so it
  can return an infeasible fallback although a feasible constrained velocity exists.
  Persistent pair state is allocated as a dense `n*n` slot matrix, including removed
  slots, and adding an unrelated agent resets all active encounters.
- Architecture documentation still contradicts the code: it calls Dijkstra “A*”, claims
  a centroid objective that the hop-count search does not optimize, and claims modules do
  not use each other's internals despite `path.cpp` including `nav_mesh_detail.hpp`.
  The plan also retains a stale 30-test verification claim.

## Implementation size and complexity

Source-only non-comment LOC is **1,292 NCLOC**, counted as every nonblank `src/` line
remaining after lexical removal of line and block comments. This excludes public headers,
tests, documentation, and generated files. The distribution is: geometry 143, shared
geometry detail 121, navmesh 145, navmesh detail 13, pathfinding 329, and simulation
**541** NCLOC.

For control-flow complexity, Clang's CFG dump is a more reliable local measure than
counting keywords or indentation. For `advance`, Clang 22 reports 151 basic blocks and
224 edges, so McCabe cyclomatic complexity is **75** (`E - N + 2`). This is configuration-
dependent and does not by itself prove a defect, but it is unusually high for one
function and supports the architecture/quality deductions. Its maximum lexical brace
nesting is **10** levels, retained as a supplementary readability signal rather than a
substitute for CFG complexity.

## Score

| Area | Score | Assessment |
|---|---:|---|
| Geometry and mesh validation | 17 / 20 | All public geometry and mesh checks pass, with deterministic topology and containment behavior. The score reserves credit for unreviewed extreme geometric cases and the repair's narrow focus. |
| Pathfinding | 17 / 20 | All public direct, disconnected, and bent-route checks pass. The deterministic Dijkstra/funnel implementation is functional, but its documentation is materially inaccurate. |
| Agent lifecycle and stepping | 12 / 20 | Public lifecycle behavior and the huge-duration underflow repair pass, but valid finite extreme radii produce non-finite state. Dense historical-slot storage also creates an avoidable resource hazard. |
| Local crowd behavior | 8 / 15 | Three of four normative crowd scenarios pass and visual motion is improved. The follower still permanently stalls in the normative corner case, a near-envelope overtake overlaps, and wide stand-off causes unnecessary dithering. |
| Tests and functional discipline | 6 / 10 | 34 deterministic native tests plus sanitizer runs are useful, but repair-02 added no regression for its replacement solver and missed the reproduced separation, finite-radius, feasibility, and encounter-lifetime defects. |
| Architecture | 5 / 10 | Geometry, navmesh, path, and simulation are separated, but the avoidance rewrite is a large, tightly coupled algorithm in `simulation.cpp`, with quadratic persistent state and stale architecture claims. |
| C++ quality | 2 / 5 | RAII/value semantics and warning-clean GCC/Clang builds are positive. Float-overflow paths, an infeasible optimization fallback, and a very dense avoidance implementation substantially reduce confidence. |
| **Total** | **67 / 100** | |

## Repair outcome

1. **First repair:** corrected tolerance-contained direct paths, T-junction handling,
large-duration substep conversion, and several overtaking/corner scenarios. It raised
public conformance to 70/72 but left crossing and close-following failure.
2. **Second repair:** replaced the avoidance mechanism and fixed public crossing,
eaching 71/72. It also corrected the previously reproduced tiny-goal/huge-duration
underflow. The new mechanism introduced untested separation and extreme-radius failures
and did not resolve the close-following terminal stall.

## Final assessment

This run improved meaningfully over its initial result and the visual behavior is often
credible. However, the second repair traded a focused local policy for a much more complex
constraint system without establishing its mathematical or numerical invariants. The
remaining corner deadlock, reproducible overlap, valid-input non-finite state, and broad
avoidance stand-off prevent it from being a dependable crowd-navigation implementation.
It is stronger than the aggressively quantized DeepSeek run because it avoids its sanitizer
failure and repairs more public behavior, but it remains clearly below the Qwen result in
robustness and maintainability.
