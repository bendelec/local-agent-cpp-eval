# Evaluation — Qwen 3.8 Flash (Q5_K_L run)

## Run identity

| Field | Value |
|---|---|
| Model | Qwen 3.8 Flash |
| Runtime | Local Lemonade / llama.cpp; Q5_K_L quantization |
| Task revision | Current VWmini task, including NFR-009 architecture and NFR-010 implementation-plan deliverables; conformance revision 2 |
| Final source | [`../solutions/qwen38-flash-q5-kl-run-01-repair-02/`](../solutions/qwen38-flash-q5-kl-run-01-repair-02/) |
| Final tree fingerprint | `d250039e8adee34a1497fad507a06dbc3b9509271aaa6f197698b67f5af72f81` |
| Repair limit | Two repair prompts |
| Final result | **75 / 100** |

## Initial state

This was an unusually strong one-shot submission. It had a coherent C++23 design, substantive
architecture and implementation-plan documentation, a clean modular split between triangulation,
mesh topology, pathfinding, steering, and mutable agent state, and 69 deterministic native tests.
It built warning-clean with GCC and Clang, passed ASan/UBSan, formatted cleanly, and linked as a
downstream consumer.

Against conformance revision 2 it passed **76/77**: geometry 15/15, navmesh/path 28/29,
lifecycle 29/29, and crowd 4/4. The only public failure was finite extreme-coordinate
containment. Independent probes also found that public `length({FLT_MAX, FLT_MAX})` returned
infinity and normalization collapsed that nonzero vector to zero; `BUILD_TESTING=OFF` was ignored.

## Repair outcome and final state

The first repair correctly introduced a centralized double-intermediate numeric policy (`Vec2d`),
fixed scale-safe containment and public vector behavior, added focused numeric regressions, and
honoured `BUILD_TESTING`. It reached **77/77** conformance. It also added an unnecessary
endpoint-plus-midpoint motion check and residual-motion integrator. Independent review reproduced
teleportation across gaps and a speed-budget violation from those additions.

The second repair deleted residual state, range-checked double-to-float narrowing, and reused the
existing mesh segment query for motion. It adds three useful regressions for the previous defects.
The resulting archive passes all advertised checks:

- **77/77** conformance: geometry 15/15, navmesh/path 29/29, lifecycle 29/29, crowd 4/4;
- **81/81** native GTest cases in Debug and Release;
- warning-clean GCC 16.2.1 and Clang 22.1.8 builds;
- GCC ASan/UBSan plus float-cast-overflow tests, with no report;
- `BUILD_TESTING=OFF` and `VWMINI_BUILD_TESTS=OFF` library-only builds;
- downstream consumer build/run; and
- `scripts/check.sh` and `clang-format --dry-run --Werror`.

Those gates are genuine strengths, but the final result still has two independently reproduced
core functional defects outside the public suite.

### Epsilon-band path and motion tunnelling

`MeshTopology::segment_is_contained` collects only intersections with geometric triangle edges,
then samples intervals between those cuts. `contains`, however, accepts a point within `epsilon`
of an edge. Entry and exit points of that epsilon-wide edge band are not geometric edge crossings,
so the interval method can miss an uncovered gap entirely.

A mesh of three disjoint CCW triangles with bases at `y = 0.00005` and centres `x = 0`, `5`, and
`10` contains `(0,0)`, `(5,0)`, and `(10,0)` through the documented boundary band, but not
`(2,0)` or `(8,0)`. The final source returns an exact direct two-point path from `(0,0)` to
`(10,0)` instead of `NoPath`. With the first triangle made tall, an overlapping idle peer can also
push a moving agent from `(0,0)` to `(10,0)` in one step while `(2,0)` and `(8,0)` remain outside.
Thus the new motion reuse is only as correct as the flawed segment primitive; it violates
SIM-001/002/003 and SIM-012 despite passing the public gap regression.

### Stored-position rounding exceeds the speed budget

The second repair removed residual accumulation but still narrows the already bounded double
endpoint to nearest `float` without rechecking the actual stored displacement. On a broad valid
mesh, start `{10000,0}`, `max_speed=1`, an eastward goal, and `step(0.0006f)` produce a stored
position displacement of `0.0009765625`, greater than the allowed `0.0006`. This violates the
SIM-008 max-speed constraint. The candidate test exercises a smaller `0.0001f` step that rounds
back to no movement, so it misses the half-to-one-ULP interval where nearest rounding jumps too
far.

Additional review findings are lower priority but real: `uint32_t next_id++` eventually emits the
prohibited zero `AgentId` and later reuses ids; and architecture text still contains contradictory
claims that Simulation uses only public mesh APIs, plus removed motion constants.

None of these probes caused a crash, a sanitizer report, or non-finite/out-of-mesh state in the
normative suite, so the 40-point safety cap does not apply. They are nevertheless substantial
functional deductions.

## Implementation size and complexity

The final archive has **1,661 implementation NCLOC** across 17 `src/` files, measured with:

```sh
./scripts/measure-source.py solutions/qwen38-flash-q5-kl-run-01-repair-02
```

The largest units are `mesh_topology.cpp` (345 NCLOC) and `simulation.cpp` (342). The split is
materially better than a monolithic navigation implementation: immutable geometry/topology,
corridor search, path construction, steering, and agent state remain independently readable.

With Clang 22.1.8 CFG dumps, the densest inspected function is `advance_agent` at 17 basic blocks
and 39 edges, McCabe **24** (`E - N + 2`); `contained_fraction` is 11, and
`MeshTopology::segment_is_contained` is 10. Lexical nesting is modest. The rubric therefore sets
a 17/20 decomposition ceiling; this evaluation scores below it for the coupled, duplicated
motion-shortening policy rather than raw size.

## Score

| Area | Score | Assessment |
|---|---:|---|
| Architecture and dependency design | 16 / 20 | Strong cohesive modules, immutable mesh ownership, clear value semantics, and a well-chosen shared numeric policy. Direct Simulation friendship to immutable topology is a narrow practical seam, but architecture text still contradicts it. |
| Decomposition and complexity | 14 / 20 | Units are reasonably bounded and the observed CFG maximum is 24, but `simulation.cpp` couples integration, float narrowing, exact coverage, and route progress. The segment-query reuse made a shared defect affect both paths and motion. |
| C++ clarity and discipline | 8 / 10 | RAII, `std::expected`, const queries, deterministic iteration, strict warnings, and checked narrowing are strong. The unchecked ID exhaustion and endpoint-rounding contract gap reduce confidence in the otherwise careful numeric policy. |
| Tests and functional discipline | 11 / 15 | 81 deterministic tests, Release activation, sanitizers, and regressions for independently found defects are substantial. The gap test shares the same faulty containment oracle, while epsilon-band connectivity and half-ULP rounding were missed. |
| Functional conformance beyond the gate | 26 / 35 | Perfect 77/77 public conformance and all crowd fixtures are meaningful. Epsilon-band direct routing/tunnelling and speed-cap violation are reproducible valid-input failures in core path and motion contracts. |
| **Total** | **75 / 100** | |

## Architecture and test observations

The best architectural decision is the centralized `Vec2d`/predicate policy: it eliminates the
original float-overflow failure without changing the fixed public representation. It is not a
premature abstraction because signed area, distance, segment intersection, containment, and
steering all genuinely share the same widening rule.

Conversely, `segment_is_contained` is a shared-but-incomplete abstraction. It is correctly
centralized, but its geometry-edge partitioning does not model the epsilon-based containment rule.
Pathfinding and the final motion safeguard both rely on it, so the same omission propagates into
two public behaviors. The second repair improved scope discipline by deleting residual state, but
its bounded bisection logic still belongs to simulation while the correctness decision remains in
mesh topology; a future correction should repair the topology interval semantics rather than add
another simulation-layer workaround.

The plan and report are unusually candid about the first-repair regressions. They nevertheless
need a final consistency pass: the architecture document retains the obsolete claim that
Simulation uses only public `NavMesh` APIs and lists deleted containment-shrink constants.

## Commands and evidence

```sh
# Public conformance
./evaluator/conformance/run.sh solutions/qwen38-flash-q5-kl-run-01-repair-02

# Native Debug / Release and library-only checks
cmake -S solutions/qwen38-flash-q5-kl-run-01-repair-02 -B /tmp/qwen38-flash-r2-native \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS='-Wall -Wextra -Wpedantic -Werror'
cmake --build /tmp/qwen38-flash-r2-native --parallel
ctest --test-dir /tmp/qwen38-flash-r2-native --output-on-failure

cmake -S solutions/qwen38-flash-q5-kl-run-01-repair-02 -B /tmp/qwen38-flash-r2-release \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/qwen38-flash-r2-release --parallel
ctest --test-dir /tmp/qwen38-flash-r2-release --output-on-failure

cmake -S solutions/qwen38-flash-q5-kl-run-01-repair-02 -B /tmp/qwen38-flash-r2-notest \
  -DBUILD_TESTING=OFF
cmake --build /tmp/qwen38-flash-r2-notest --parallel

# Sanitizer and format checks
cmake -S solutions/qwen38-flash-q5-kl-run-01-repair-02 -B /tmp/qwen38-flash-r2-san \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined,float-cast-overflow -fno-omit-frame-pointer'
cmake --build /tmp/qwen38-flash-r2-san --parallel
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1 \
  ctest --test-dir /tmp/qwen38-flash-r2-san --output-on-failure
clang-format --dry-run --Werror $(find solutions/qwen38-flash-q5-kl-run-01-repair-02 \
  -type f \( -name '*.cpp' -o -name '*.hpp' \) | sort)
```

The independent epsilon-band and rounding probes were compiled only against the final archive and
executed from `/tmp`; they did not modify the candidate or source archive.

## Final assessment

Qwen 3.8 Flash produced the strongest public behavior of the evaluated local candidates so far:
it starts with all four crowd cases working and reaches full normative conformance after a focused
numeric repair. Its internal organization, build discipline, and error handling are also strong.

The final score is held below release-quality by a subtle but important mismatch between exact
geometric-edge interval logic and epsilon-tolerant public containment, plus a remaining
float-rounding speed violation. These are the kinds of defects a library must resolve before its
otherwise credible architecture and excellent public-suite outcome can be trusted generally.
