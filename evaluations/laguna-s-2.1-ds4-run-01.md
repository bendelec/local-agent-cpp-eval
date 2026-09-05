# Evaluation — Poolside Laguna S 2.1 (DS4 run)

## Run identity

| Field | Value |
|---|---|
| Model | Poolside Laguna S 2.1 |
| Runtime | Local Antirez DS4; exact serving build/version not captured. |
| Run label | `laguna-s-2.1-ds4-run-01` |
| Task revision | Current VWmini task, including NFR-009 architecture and NFR-010 implementation-plan deliverables |
| Final source | [`../solutions/laguna-s-2.1-ds4-run-01-repair-02/`](../solutions/laguna-s-2.1-ds4-run-01-repair-02/) |
| Final tree fingerprint | `c3f642cb6a624684e7aa05a87c0a4756815a476942ad83bf6dd0058b4e6b4bb9` |
| Repair limit | Two repair prompts |
| Final result | **51 / 100** |

## Serving context

At the time of this DS4 run, serving support for Laguna S 2.1 was unsatisfactory.
Earlier Lemonade and Poolside `llama.dpp` attempts failed for inference/serving-stack
reasons rather than observed model behavior. This run became viable after reviving the
[DS4 Laguna branch](https://github.com/bendelec/ds4) on current `main` and adding a simple
attractor-loop guardrail. The resulting implementation remains unsatisfactory for the
engineering reasons documented below.

## Final verification

The archived final tree configured and built successfully as C++23 with GCC 16.2 and
Clang 22.1.8. Its own deterministic test executable passed all five test groups under
both compilers; GCC AddressSanitizer/UndefinedBehaviorSanitizer also passed without a
report. A library-only configuration (`-DBUILD_TESTING=OFF`) builds successfully. The
fixed public declarations, defaults, enum values, and target alias are unchanged; the
only public-header differences are layout.

The final normative result is **71 / 77**:

| Track | Result | Material failures |
|---|---:|---|
| Geometry | 15 / 15 | — |
| Navmesh/path | 27 / 29 | visible multi-cell route is not exactly direct; irregular connected-mesh route leaves walkable space |
| Simulation lifecycle | 28 / 29 | bent route around a reflex corner does not reach its goal |
| Crowd | 1 / 4 | crossing, overtaking, and close reflex-corner following fail; only overlap recovery passes |

The failure is substantive rather than a test-harness issue. In the crossing and
overtaking fixtures, the agents fail to reach their goals and their separation drops as
low as **0.4834** where the required minimum is **0.499**. In the close-following L
fixture it drops to about **0.4635**, and both agents remain `Moving` after ten seconds.
The lifecycle bent-route agent remains about **0.7454 m** from a goal that has a
0.01 m arrival radius.

`clang-format --dry-run --Werror` with the submitted `.clang-format` fails across public
headers, production code, and tests. This contradicts the final-verification claim in
the implementation plan. GCC and Clang compilation itself was warning-clean under the
project's `-Wall -Wextra -Wpedantic` options.

## Reproduced findings

### High — paths do not meet their basic containment/directness contract

`find_path` uses a same-cell-only direct shortcut (`src/nav_mesh.cpp:385-388`), followed
by centroid-weighted cell Dijkstra and an incorrectly oriented custom funnel
(`:205-341`, `:396-428`). It therefore returns extra points for a straight segment across
multiple cells, contrary to SIM-001. More seriously, the irregular connected-mesh
conformance fixture returns a polyline segment through non-walkable space, violating
SIM-002 and SIM-003. These are the two failed navmesh/path tests.

The implementation should first use an exact segment-in-union-of-triangles predicate for
direct visibility, then replace or validate the corridor/funnel result so every emitted
segment is continuously covered by the mesh.

### High — avoidance violates the required normal-crowd behavior

The pairwise velocity adjustment in `src/simulation.cpp:347-375` only corrects the
currently processed pair and then independently clamps each speed. Subsequent pair
processing, clamping, and containment projection invalidate the earlier pair constraint.
There is no post-integration separation repair at `:378-409`, despite the architecture
claim. The three failed crowd scenarios demonstrate both collision and progress failures
on the small feasible cases explicitly required by SIM-011.

### High — waypoint/arrival handling stalls valid routes

At `src/simulation.cpp:388-409`, displacement is not limited to the current waypoint;
arrival is tested only after the agent moves. Together with the unvalidated funnel route,
this leaves the reflex-corner lifecycle agent projected at a blocked boundary rather than
at its goal. The same motion ordering independently fails a simple additional valid-input
probe: in the open square mesh, an agent from `(1,5)` to `(5.03,5)`, speed `2`, and
arrival radius `0` remains `Moving` after `step(100)`, oscillating near `(5.003,5)`
instead of reaching the exact goal. This violates SIM-008 and SIM-009's
no-overshoot/stable-arrival contract.

### Medium — a valid non-degenerate polygon is rejected

`src/triangulator.cpp:155-160` removes every ear whose local cross product is at most
`1e-4`, although the specified non-degeneracy threshold is `epsilon² = 1e-8`.
`{{0,0}, {0.01,0}, {0,0.008}}` has signed double area `8e-5`, well above `epsilon²`, and
its distinct vertices are farther apart than epsilon; nevertheless the function returns
`InvalidMesh` (`"no triangles produced"`). This violates MSH-002/003.

### Medium — finite-vector semantics are not scale safe

`length` is `sqrt(dot(v,v))` (`src/geometry.cpp:9-12`). For the finite vector
`{1e30f,0}`, the float multiplication overflows: `length` is infinity and `normalized`
returns `{0,0}`. That is not the usual vector meaning required by MSH-001, and also
contradicts the public header's promise that finite input yields a finite length. A
scale-safe norm (for example `std::hypot`) and scaled normalization are needed.

### Medium — design documentation and claimed verification do not describe the submission

The architecture says the pathfinder is `src/path_finder.cpp`, uses a visibility graph,
and has `SegmentCover` (`docs/architecture/architecture.md:15-17, 50-70`). None exists:
the submitted implementation is a centroid corridor plus funnel in `src/nav_mesh.cpp`.
The implementation plan repeats those nonexistent components (`implementation-plan.md:48-54,
89-90`), says waypoint clamping and residual positional overlap correction are done
(`:72-73`), and claims a clean formatting sweep (`:80-85`). These claims are all
contradicted by the final source and verification. This materially misses NFR-009 and
NFR-010, rather than being a cosmetic stale comment.

The README also still tells the reader that the architecture documents are absent and
must be created, though they are present. The CMake comment calls testing opt-in while
`BUILD_TESTING` defaults to `ON`.

## Maintainability evidence

Production source is approximately **1,026 NCLOC** under a consistent lexical
comment-removal count: geometry 17, triangulation 154, navmesh 361, private navmesh
details 128, and simulation 365. Clang CFG output for `Simulation::step` has **68 basic
blocks**, **97 edges**, and McCabe complexity **31**. That stays below the rubric's
objective complexity ceiling of 17/20, but the function still combines substep scheduling,
desired motion, pair constraint handling, integration, containment repair, and route-state
transitions. Its observed bugs occur at those policy boundaries.

There are positive structural choices: immutable PIMPL-backed mesh data, clear ownership
through `shared_ptr<const Impl>` and `unique_ptr<Impl>`, value snapshots, transactional
validation for `add_agent`/`set_goal`, no global mutable state, and a compact dependency
direction from geometry through mesh/path to simulation. Those benefits are reduced by
the incorrect architecture record, duplicated float geometry policy, and the central
stepping routine's coupled responsibilities.

The submitted unit suite is deterministic and exercises many validation paths, but its
single aggregate executable passes while missing a multi-cell directness case beyond its
two-triangle fixture, continuous irregular-path containment, the bent route that now
fails, all requested normal-crowd
scenarios, zero-radius non-aligned arrival, small valid triangles, and large finite vector
inputs. Thus it supplies useful baseline discipline, not reliable independent coverage of
the risk areas.

## Score

No 40-point safety cap applies. No sanitizer report, crash, non-finite agent state, or
out-of-mesh agent position was reproduced in normative conformance. The valid-input
correctness defects above remain substantial functional deductions.

| Area | Score | Assessment |
|---|---:|---|
| Architecture and dependency design | 12 / 20 | Good immutable ownership and component boundaries, but the required architecture/plan documents describe a different program and overstate guarantees. |
| Decomposition and complexity | 12 / 20 | The CFG-31 result remains below the objective ceiling, yet `step` couples too many policy stages; its constraints are not preserved across those stages. |
| C++ clarity and discipline | 5 / 10 | RAII, values, `expected`, const queries, and warning-clean builds are positives. Formatting is not clean and float overflow plus the invalid area threshold undermine numeric discipline. |
| Tests and functional discipline | 5 / 15 | Deterministic local tests and sanitizer runs exist, but they miss each core final path/crowd regression and final verification claims are inaccurate. |
| Functional conformance beyond the gate | 17 / 35 | 71/77 is a meaningful partial implementation, but invalid paths, one lifecycle stall, and failure of all normal-crowd scenarios prevent dependable use. |
| **Total** | **51 / 100** | |

## Commands used

```sh
# Normative conformance
evaluator/conformance/run.sh \
  solutions/laguna-s-2.1-ds4-run-01-repair-02 \
  /tmp/laguna-ds4-r2-eval-conformance

# Native and sanitizer checks
cmake -S solutions/laguna-s-2.1-ds4-run-01-repair-02 \
  -B /tmp/laguna-ds4-r2-eval-native -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/laguna-ds4-r2-eval-native --parallel
ctest --test-dir /tmp/laguna-ds4-r2-eval-native --output-on-failure

cmake -S solutions/laguna-s-2.1-ds4-run-01-repair-02 \
  -B /tmp/laguna-ds4-r2-eval-san -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
  -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined'
cmake --build /tmp/laguna-ds4-r2-eval-san --parallel
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1 \
  ctest --test-dir /tmp/laguna-ds4-r2-eval-san --output-on-failure
```

## Final assessment

The two repairs preserved the public API and produced a compilable, safe-on-tested-inputs
library with credible lifecycle validation and overlap recovery. But the final pairwise
velocity projection fails every normal-crowd acceptance scenario, while the underlying
funnel remains invalid on general meshes. The mismatch between polished design claims and
what is compiled is a second-order reliability concern: the missing direct-visibility,
segment-cover, waypoint-clamping, and positional-correction pieces are exactly the pieces
the documents assert exist. This is a useful partial implementation, not a dependable
navigation/crowd library.
