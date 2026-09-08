# Final Evaluation — GPT-5.6 terra

## Run identity and provenance

| Field | Value |
|---|---|
| Model / runtime | GPT-5.6 terra / OpenAI Codex API subscription, `gpt-5.6-terra` |
| Task | Current VWmini task with NFR-009/NFR-010; conformance revision 3 (82 tests) |
| Initial archive | [`../solutions/gpt56-terra-openai-codex-run-01/`](../solutions/gpt56-terra-openai-codex-run-01/), `460a62ec1187f329df1a02f9a4420484cb46646cce72409b309bba6bf0496c2c` |
| Repair 01 archive | [`../solutions/gpt56-terra-openai-codex-run-01-repair-01/`](../solutions/gpt56-terra-openai-codex-run-01-repair-01/), `8e45304b547cb9054ce5a57817f63f89d582134c9ad0cc8467a082e43a328956` |
| Final archive | [`../solutions/gpt56-terra-openai-codex-run-01-repair-02/`](../solutions/gpt56-terra-openai-codex-run-01-repair-02/), `1c513b3038876b1d1736825b50b5a0d1ee56ac8faba25f3a565950f257dd8d6a` |
| Final state | Evaluated; no further repair round |

Each archive was copied from the completed mutable workspace while omitting generated build trees
and repository metadata. A post-copy source comparison of the final archive was empty. The final
archive contains only candidate task source, tests, requirements, and architecture documents.

## Result: **40/100 — safety-capped**

The ordinary rubric subtotal is **72/100**, but the published safety gate caps the final result at
**40**. Both public movable value types can be queried on their moved-from source object and dereference a null pImpl,
causing a reproducible sanitizer-reported null access/crash. `NavMesh` permits compiler-generated moves, `Simulation` explicitly default-declares moves, and no contract
narrows public query use on a moved-from source object; this violates the required
RAII/value-semantics safety. It is therefore a demonstrated core safety failure, not merely a
stylistic deduction.

| Rubric area | Score | Basis |
|---|---:|---|
| Architecture and dependency design | 17 / 20 | Clear geometry/immutable-mesh/simulation boundaries, private pImpl ownership, no global mutable state or needless framework. The remaining documentation overstates transactional `Impl` construction, and the crowd policy is still tightly coupled to integration. |
| Decomposition and complexity | 11 / 20 | `NavMesh::create` still combines validation, overlap/T-junction checks, edge pairing, and component labelling in 124 lines (clang-tidy cognitive complexity 91). Crowd selection is likewise a 45-complexity policy routine. The maximum observed lexical nesting is six, so the rubric's objective ceiling is 17; the responsibilities reduce the score below it. |
| C++ clarity and discipline | 8 / 10 | Warning-clean, formatted C++23 with sensible standard-library/value use and readable names. Null moved-from state and a one-ULP reported-velocity overspeed prevent full credit. |
| Tests and functional discipline | 10 / 15 | Deterministic candidate regressions cover the repaired finite arithmetic, area threshold, extreme routing, and one L-passage scenario. They miss move-state safety and use a materially easier close-following case than the still-failing required scenario. |
| Functional conformance beyond gate | 26 / 35 | 81/82 public tests pass: geometry 15/15, navmesh 30/30, simulation 33/33, crowd 3/4. The remaining crowd case is a substantive valid-input failure; a small speed-boundary defect also remains. |
| **Ordinary subtotal** | **72 / 100** | **Capped at 40 by the core safety failure.** |

## Conformance and validation evidence

| Track | Initial | Repair 01 | Final |
|---|---:|---:|---:|
| Geometry | 15/15 | 15/15 | **15/15** |
| Navmesh/path | 30/30 | 30/30 | **30/30** |
| Simulation | 31/33 | 33/33 | **33/33** |
| Crowd | 2/4 | 3/4 | **3/4** |
| **Total** | **78/82** | **81/82** | **81/82** |

Final validation used the immutable repair-02 archive:

- GCC 16.2.1 Debug and Release, each with `-Wall -Wextra -Wpedantic -Werror`: configure,
  build, and candidate CTest **1/1 passed**.
- Clang 22.1.8 Debug with the same warnings-as-errors: candidate CTest **1/1 passed**.
- Clang Debug ASan/UBSan/float-cast-overflow: candidate CTest **1/1 passed** with no report.
  This suite did not exercise moved-from values; the focused ASan/UBSan probe below does.
- `BUILD_TESTING=OFF` GCC Release library-only build passed and correctly has no test target.
- `clang-format --dry-run --Werror` passed. `cppcheck` was not installed in the evaluation
  environment.
- Public conformance was **81/82**. The sole failure is
  `Crowd_ReflexCornerFollowing.CloseAgentsBothRoundCornerAndReach`: after 600 calls to
  `step(1/60)`, the follower remains `Moving` rather than `Reached`.

Measured implementation size is 1,274 physical NCLOC (non-comment implementation lines) across
`geometry.cpp` (166), `nav_mesh.cpp` (467), and `simulation.cpp` (641).

## Material remaining defects

1. **Core safety: moved-from values crash.** `NavMesh` owns a `shared_ptr` and `Simulation` a
   `unique_ptr`, but their defaulted moves empty those pointers while public queries dereference
   them unconditionally (`src/nav_mesh.cpp:464-477`, `src/simulation.cpp:711-721`). A focused
   ASan/UBSan program that creates a mesh, moves it, then calls `original.cell_count()` crashes at
   `NavMesh::cell_count` with a null member access. The analogous moved-from
   `Simulation::agent_count()` has the same defect. A robust value type must retain a query-safe
   post-move invariant or guard and define these query results.
2. **SIM-010/SIM-011/SIM-012: close followers deadlock at a reflex corner.** The final repair
   correctly avoids the prior overlap and prevents a mesh-clamped endpoint from losing all route
   reachability, but the normative radius-0.25, speed-1.4 close-following fixture still leaves its
   follower stationary and `Moving`. The local candidate selector lets the leading agent reach in
   the narrow arm and then has no passing/progress configuration for the follower before the
   10-second budget. The candidate's passing L-passage regression uses radius 0.125, speed 1.0,
   different coordinates, and 18 seconds, so it does not establish this required case.
3. **SIM-008 boundary precision: reported velocity can slightly exceed `max_speed`.**
   `committed_motion` validates the double stored displacement, then narrows its velocity
   components without a final magnitude check (`src/simulation.cpp:221-234`). On a broad mesh,
   an agent with `max_speed=117.7760391f`, goal `{23.47055626f,97.20664978f}`, and
   `step(0.006346113514f)` reports magnitude `117.776041085`, above its stored maximum
   `117.776039124`. This is small but violates the stated bound.

The architecture document also says construction creates `NavMesh::Impl` only after validation;
source allocates it before validation (`src/nav_mesh.cpp:351`). This is documentation inaccuracy,
not a transactionality failure visible to callers.

## What the repair rounds achieved

The initial submission had four public failures: tiny-duration non-finite velocity, stored-position
speed excess, crowd crossing non-completion, and close reflex-corner following/overlap. Repair 01
resolved the first three and improved the latter to a no-overlap but stalled follower. The final
repair made finite geometry predicates double-based, fixed signed-area comparison units, made
normalization robust at denormal/maximum finite magnitudes, retained only route-continuing
mesh-clamped endpoints, and added substantial deterministic regressions. Those changes preserve
all geometry, navmesh, and simulation conformance tests and eliminate the earlier extreme-scale
routing and terminal-motion collision concerns. The unresolved required crowd case and the newly
observed moved-from safety violation determine the final grade.

## Reproduction commands

```sh
# Final public result
./evaluator/conformance/run.sh solutions/gpt56-terra-openai-codex-run-01-repair-02 \
  /tmp/gpt56-terra-r2-conformance

# Candidate Debug test build
cmake -S solutions/gpt56-terra-openai-codex-run-01-repair-02 -B /tmp/gpt56-terra-r2-debug \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS='-Wall -Wextra -Wpedantic -Werror'
cmake --build /tmp/gpt56-terra-r2-debug --parallel
ctest --test-dir /tmp/gpt56-terra-r2-debug --output-on-failure
```
