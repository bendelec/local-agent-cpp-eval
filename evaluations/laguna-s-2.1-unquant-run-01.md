# Evaluation — Poolside Laguna S 2.1 (unquantized)

## Run identity

| Field | Value |
|---|---|
| Model | Poolside Laguna S 2.1 |
| Runtime | Hosted OpenRouter API; `poolside/laguna-s-2.1` |
| Task revision | Current VWmini task, including NFR-009 architecture and NFR-010 implementation-plan deliverables |
| Final source | [`../solutions/laguna-s-2.1-unquant-run-01-repair-02/`](../solutions/laguna-s-2.1-unquant-run-01-repair-02/) |
| Final tree fingerprint | `d7e6136cc1de8c7e35207c73e3e7e893df07b5f213cbb6ed5a1ff918ef6bd5d2` |
| Repair limit | Two repair prompts |
| Final result | **63 / 100** |

## Initial state

The initial hosted submission was substantive and unusually well structured for a first
pass. It preserved all public declarations, split implementation into geometry,
triangulation, navmesh, path, and simulation units, and provided architecture documents
and a native test suite. Its native tests passed **45/45** under sanitizers.

Initial public conformance was **66/72**: geometry 15/15, navmesh/path 23/24, simulation
lifecycle 27/29, and crowd 1/4. The important failures were an invalid shortcut through a
reflex corner, non-transactional failed `add_agent`, and ordinary crowd interactions that
never completed.

## Repair outcome and final state

Both repair passes made real, targeted progress. Repair 1 corrected the reflex-corner
fixture, transactional add, vertex-only mesh-contact policy, formatter configuration, and
several lifetime/duration cases, raising conformance to **69/72**. Repair 2 replaced the
normal-only avoidance projection with bounded deterministic velocity candidates and
best-response selection. It repaired crossing, overtaking, overlap recovery, and close
reflex-corner following in the public suite.

At finalization, the source passed all **72/72** then-current conformance tests (geometry
15/15, navmesh/path 24/24, lifecycle simulation 29/29, and crowd 4/4). Conformance revision
2 added five navmesh/path cases and reran the archived final snapshot: it now passes
**74/77** (geometry 15/15, navmesh/path 26/29, lifecycle 29/29, crowd 4/4). It fails the
finite-extreme-coordinate containment, multi-cell direct-route, and irregular-mesh
continuous-containment tests.

Its native suite passes **52/52** with GCC and Clang; an independent GCC ASan/UBSan run
also passes 52/52. GCC and Clang builds were warning-clean, and clang-format check mode
passes. Only whitespace/layout changed in two public headers; declarations, default
arguments, types, and linkage-relevant public definitions remain stable.

This result is not sufficient for release quality. Independent final review and direct
probes reproduced the following material valid-input defects; the first three now have
revision-2 normative coverage:

- **A returned path can leave the mesh.** A deterministic irregular 5×5 cell mesh with
  occupied cells (top row first) `##### / ###.# / ##### / #.### / #####`, queried from
  `(1.31, 0.69)` to `(1.69, 2.31)`, returns
  `[(1.31,0.69), (2,1), (1.69,2.31)]`; sampling the last segment finds non-walkable
  space. The custom funnel's portal orientation/cone updates are not generally valid.
  This violates the continuous-containment path contract.
- **Straight multi-cell paths are not reliably direct.** In a fully walkable 5×5
  triangulated rectangle, the contained segment from approximately `(4.826,4.495)` to
  `(2.735,0.870)` returns three points rather than exactly `[start, goal]`. The code
  performs a same-cell shortcut only and lacks the required whole-segment direct-path
  check.
- **Finite extreme-coordinate meshes are numerically unusable.** The revision-2 fixture
  creates `{(0,0), (FLT_MAX/4,0), (0,FLT_MAX/4)}` and finds its interior point
  `(FLT_MAX/16, FLT_MAX/16)` not contained. Float squared lengths and cross products
  overflow. This also contradicts the submitted mesh documentation's claim of double-area
  handling.
- **Waypoint indexing has a credible out-of-bounds route.** A moving agent can advance
  `target_index` past its final route point after coming within epsilon of it, while a
  zero arrival radius prevents `Reached`; the next substep's motion phase indexes
  `route[target_index]` without a bound check. The final sanitizer suite does not cover
  this avoidance/clamping/zero-radius combination.

These findings do not trigger the rubric's 40-point safety cap: no sanitizer-reported
memory error, crash, or non-finite/out-of-mesh state was reproduced in normative
conformance. They are nevertheless substantial deductions from functional correctness.

Reproduction used:

```sh
evaluator/conformance/run.sh \
  solutions/laguna-s-2.1-unquant-run-01-repair-02 /tmp/laguna-r2-conformance
cmake -S solutions/laguna-s-2.1-unquant-run-01-repair-02 \
  -B /tmp/laguna-r2-native -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/laguna-r2-native --parallel
ctest --test-dir /tmp/laguna-r2-native --output-on-failure
```

## Implementation size and complexity

Source-only non-comment LOC is **971 NCLOC**, counted as nonblank `src/` lines after
lexical removal of line and block comments. This excludes public headers, tests,
documentation, and generated files. `simulation.cpp` accounts for **460 NCLOC**; the
remaining production code is distributed across focused geometry (16), navmesh (178),
path (210), and triangulation (105) units.

Clang 22 CFG output for `Simulation::step(float)` has **108 basic blocks** and **173
edges**, giving McCabe complexity **67** (`E - N + 2`). The routine combines live-agent
collection, timestep policy, candidate generation, neighbour candidate expansion,
iterative selection, movement, waypoint handling, and termination in one function. Under
the rubric's objective complexity ceiling (51–70), the decomposition category cannot
score above 8/20.

## Score

| Area | Score | Assessment |
|---|---:|---|
| Architecture and dependency design | 16 / 20 | The PIMPL-backed public types and separate geometry, mesh, path, and simulation responsibilities are strong. The private `NavMesh::Impl` seam is sensible. Simulation remains too policy-dense, but ownership and mutability are clear. |
| Decomposition and complexity | 6 / 20 | `Simulation::step` is a 280-line, CFG-67 orchestration and algorithm implementation. Candidate generation, geometric collision scoring, route state, containment, and integration should be separate cohesive helpers. |
| C++ clarity and discipline | 7 / 10 | RAII, values, `expected`, const queries, deterministic tie-breaking, and warning-clean builds are positives. Dense nested local lambdas/vectors and unguarded route indexing reduce auditability; overflow-prone float predicates weaken numeric discipline. |
| Tests and functional discipline | 10 / 15 | The 52 deterministic tests, both repair rounds, compiler checks, and sanitizers are meaningful. Tests missed the final crowd-completion scenarios requested in repair 2, general funnel containment/directness, extreme finite coordinates, and the zero-radius final-waypoint path. |
| Functional conformance beyond the gate | 24 / 35 | Revision-2 conformance is 74/77 and ordinary crowd behavior improved dramatically. The promoted funnel, direct-route, and finite-coordinate failures were already material deductions at final scoring. |
| **Total** | **63 / 100** | |

## Documentation and build notes

The repair work made the formatter configuration usable and substantially improved stale
status documentation. Some final text still needs care: architecture and completion
reports claim `seconds / target_substep` is finite merely because `seconds` is finite,
which is false for `FLT_MAX / 0.05f` even though the implementation's comparison avoids
the unsafe cast. The architecture also says avoidance is "deadlock-free by construction,"
a stronger claim than the local algorithm and requirements establish. The implementation
plan's WP7 language says `clang-tidy` is clean, while its final verification note correctly
says it was not configured/run.

The default CMake configuration also requires GTest because testing defaults to `ON`.
Library-only configuration works with `-DVWMINI_ENABLE_TESTING=OFF`, but a consumer should
not need a test-only third-party package merely to configure the library.

## Final assessment

This is a capable iterative result: the model built a real navigation/crowd library,
responded effectively to two focused repair prompts, preserved the public interface, and
reached full then-current conformance. Its core weakness is that success on fixtures led to
a large bespoke central avoidance routine while the custom funnel remained insufficiently
general. Revision 2 makes those previously hidden path defects explicit in conformance. The invalid-path reproduction is especially important because it defeats a basic
purpose of a navigation mesh despite passing the then-current 24-test navmesh/path suite.
The submission is therefore a promising but not dependable small navigation library: clearly
stronger than
most incomplete or safety-capped runs, but below a release-quality pathfinding and
simulation implementation.
