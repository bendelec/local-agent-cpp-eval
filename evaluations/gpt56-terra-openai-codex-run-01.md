# Evaluation — GPT-5.6 terra (initial round)

## Run identity

| Field | Value |
|---|---|
| Model | GPT-5.6 terra |
| Runtime | OpenAI Codex API subscription; `gpt-5.6-terra` |
| Task revision | Current VWmini task including NFR-009/NFR-010; conformance revision 3 (82 tests) |
| Initial source | [`../solutions/gpt56-terra-openai-codex-run-01/`](../solutions/gpt56-terra-openai-codex-run-01/) |
| Initial tree fingerprint | `460a62ec1187f329df1a02f9a4420484cb46646cce72409b309bba6bf0496c2c` |
| State | Repair 01 evaluated; final repair requested; final score deferred |

The immutable initial archive was copied from the completed external workspace while omitting its
single generated `build/` tree. A post-copy source comparison and the fingerprint above matched
the candidate model-authored tree. The archive contains only task source, tests, requirements,
and required architecture documents; no evaluator/reference material was added.

## Initial result

**Build/API gate: PASS.** The CMake project exposes `vwmini::vwmini`, requires C++23, and builds
with GCC 16.2.1 and Clang 22.1.8 under `-Wall -Wextra -Wpedantic -Werror`. The only header deltas
from the supplied files are private friend declarations needed by the implementation; fixed public
signatures, defaults, enums, includes, and target name remain intact.

**Safety gate: PASS.** Native ASan/UBSan candidate tests reported no memory or undefined-behavior
finding. The public conformance run demonstrates a finite-state defect for the smallest finite
positive duration, but it is not a crash, sanitizer finding, or out-of-mesh state; it is a
substantial SIM-008/SIM-012 functional defect rather than an automatic 40-point-cap event.

**Conformance tracks:** G **15/15**, N **30/30**, S **31/33**, C **2/4**; total **78/82**.

| Track | Result | Initial evidence |
|---|---:|---|
| Geometry | 15/15 | All geometry/triangulation conformance tests pass. |
| Navmesh/path | 30/30 | All topology, tolerance, direct, bent, deterministic, and continuous-containment cases pass. |
| Simulation | 31/33 | Fails stored-position speed budget and smallest-positive-duration finite velocity. |
| Crowd | 2/4 | Overtaking and overlap recovery pass; crossing and close reflex-corner following fail to complete, and the latter also has slight material overlap. |

### Material defects / unmet requirements

- **SIM-008, SIM-012:** At `{10000,0}`, `max_speed=1`, `step(0.0006f)` advances the observed
  stored position by `0.0009765625`, greater than the `0.0006` speed allowance. Float endpoint
  rounding is not bounded by actual stored displacement.
- **SIM-008, SIM-012:** `step(std::numeric_limits<float>::denorm_min())` leaves a moving agent
  with NaN velocity. `displacement * (1.0F / seconds)` overflows in the reciprocal.
- **SIM-010, SIM-011, SIM-012:** The feasible two-agent crossing scene leaves both agents
  `Moving` after its 12-second budget. An independent trace ends near `x=7.979`, with agents
  trapped by their local separation response instead of reaching distinct goals.
- **SIM-010, SIM-011, SIM-012:** In close reflex-corner following, two valid agents reach a
  separation of about `0.49849` when the allowed minimum is `0.499`; the follower subsequently
  stalls at the inside corner and remains `Moving` after 10 seconds.
- **NFR-004, NFR-007:** The overall dependencies are sound, but `NavMesh::create` combines all
  validation/topology/component stages in 123 lines (clang-tidy cognitive complexity 92). The
  stepping helper also couples collision correction, containment clipping, integration, and route
  state transitions. The tree supplies no clang-format configuration, and the available default
  `clang-format --dry-run --Werror` reports every C++ file unformatted. The single 145-line custom
  assertion executable lacks regressions for all four public defects.

## Architecture and plan observations

The design documentation matches the source: geometry owns polygon validation/triangulation;
`NavMesh` holds immutable shared mesh data, containment, topology, and deterministic visibility
routing; `Simulation` uniquely owns mutable agents via pImpl. The inline data-flow diagram,
value/shared ownership description, concrete trade-off for a visibility graph, and ordered work
packages are credible. There are no threads, global mutable state, third-party dependencies, or
speculative framework layers.

The repair should preserve this direct decomposition while isolating the concrete stages of mesh
construction and simulation motion. The existing claim of “predicted disc separation” is not
strong enough: it must hold for committed, containment-limited, float-stored endpoints. The
implementation plan should record the repair rather than leave its quality work package marked
complete without the missing checks.

## Commands and toolchain

```sh
# Native Debug and Release, GCC 16.2.1
cmake -S solutions/gpt56-terra-openai-codex-run-01 -B /tmp/gpt56-terra-native-debug \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS='-Wall -Wextra -Wpedantic -Werror'
cmake --build /tmp/gpt56-terra-native-debug --parallel
ctest --test-dir /tmp/gpt56-terra-native-debug --output-on-failure
# Result: 1/1 candidate test passed.
# Equivalent Release build/test: 1/1 passed.

# Clang 22.1.8 and sanitizer evidence
CC=clang CXX=clang++ cmake -S solutions/gpt56-terra-openai-codex-run-01 \
  -B /tmp/gpt56-terra-clang -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-Wall -Wextra -Wpedantic -Werror'
cmake --build /tmp/gpt56-terra-clang --parallel
ctest --test-dir /tmp/gpt56-terra-clang --output-on-failure
# Result: 1/1 passed.
# Clang ASan/UBSan Debug candidate test: 1/1 passed; no report.

cmake -S solutions/gpt56-terra-openai-codex-run-01 -B /tmp/gpt56-terra-library-only \
  -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS='-Wall -Wextra -Wpedantic -Werror'
cmake --build /tmp/gpt56-terra-library-only --parallel
# Result: library build passed and vwmini_tests target is absent.

./evaluator/conformance/run.sh solutions/gpt56-terra-openai-codex-run-01 \
  /tmp/gpt56-terra-conformance
# Result: 78/82; G 15/15, N 30/30, S 31/33, C 2/4.
```

## Supplemental source review

- Implementation NCLOC: **1,031** across three `src/*.cpp` files, measured with
  `./scripts/measure-source.py solutions/gpt56-terra-openai-codex-run-01`.
- Clang-tidy 22.1.8 audit: cognitive complexity 37 for triangulation, 29 for segment containment,
  26 for component routing, 29 for predicted-overlap resolution, and **92** for
  `NavMesh::create`; its broad style suggestions were not treated as requirements.
- Duplication/unification: local finite/orientation/error helpers recur in geometry and navmesh;
  they are currently small and domain-local. The more material concern is not duplication but
  simulation’s conflation of intended velocity, collision-corrected velocity, mesh-feasible
  displacement, stored endpoint, and reported velocity.

## Repair request

The required focused repair is recorded in
[`repair-prompts/gpt56-terra-openai-codex-run-01-repair-01.md`](repair-prompts/gpt56-terra-openai-codex-run-01-repair-01.md).

## Repair 01 outcome

Repair 01 is archived at
[`../solutions/gpt56-terra-openai-codex-run-01-repair-01/`](../solutions/gpt56-terra-openai-codex-run-01-repair-01/),
fingerprint `8e45304b547cb9054ce5a57817f63f89d582134c9ad0cc8467a082e43a328956`.
It correctly added a checked formatting configuration, a `BUILD_TESTING=OFF`-honouring build,
double-based stored endpoint accounting, finite tiny-duration stepping, and stronger project
regressions. Fresh GCC Debug/Release tests, Clang tests, Clang ASan/UBSan/float-cast-overflow
tests, the format check, and a library-only build all pass. The repair also raises public results
to **81/82**: G 15/15, N 30/30, S 33/33, C 3/4.

The remaining public failure is close reflex-corner following: the leader reaches but the follower
remains `Moving` with zero velocity after the time budget. A trace places it at the tolerance
fringe near `{1.0001,1.0001}`; a fresh route query from that clamped position cannot reach either
its original portal or goal. Thus a mesh safety clamp can strand a valid route instead of
preserving route progress.

Independent probes also found defects outside the current public suite:

- a finite L-shaped mesh at `FLT_MAX` scale accepts a direct path through its unwalkable missing
  corner because `segment_contained` performs `Vec2` subtraction and `dot`/`cross` in `float`
  before widening;
- literal triangles and ear clipping compare twice-area `orient` to the signed-area threshold,
  accepting a triangle of signed area `7.5e-9` although `epsilon * epsilon` is `1e-8`; and
- `normalized({denorm_min, 0})` produces `{inf, NaN}`, while a maximum-finite diagonal collapses
  to `{0,0}`.

The simulation refactor is clearer about stored motion but grew to 619 physical source lines, and
`select_separated_motions` has clang-tidy cognitive complexity 45. `NavMesh::create` remains 92.
These are maintainability concerns rather than a gate, but the final repair should avoid another
broad policy layer.

No crash, sanitizer report, or normative non-finite/out-of-mesh state was reproduced; the safety
cap still does not apply. A final repair is necessary and is specified in
[`repair-prompts/gpt56-terra-openai-codex-run-01-repair-02.md`](repair-prompts/gpt56-terra-openai-codex-run-01-repair-02.md).
Final scoring remains deferred.
