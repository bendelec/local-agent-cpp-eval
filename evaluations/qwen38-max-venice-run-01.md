# Evaluation — Qwen 3.8 Max (initial round)

## Run identity and archive

| Field | Value |
|---|---|
| Model | Qwen 3.8 Max |
| Runtime | Venice API; `qwen-3-8-max` |
| Task revision | Current VWmini task including NFR-009/NFR-010; conformance revision 3 (82 tests) |
| Initial source | [`../solutions/qwen38-max-venice-run-01/`](../solutions/qwen38-max-venice-run-01/) |
| Initial tree fingerprint | `df1985468ee3ecc7d7599d37562a5015c6acf2c45bc61b357f153497f4e63647` |
| State | Initial round evaluated; repair 01 requested; final score deferred |

The completed external workspace had an observed clean Git repository with **13** logical
conventional commits, from scaffold through implementation, tests, formatting, review fixes, and
final documentation (HEAD `96a071f`). The sanitized, non-reproducible source-workspace observation
is retained in [`../sessions/qwen38-max-venice-run-01/intake.md`](../sessions/qwen38-max-venice-run-01/intake.md).
It is a genuine process strength and the first observed candidate workspace in this series to retain
such an incremental project history. The immutable source archive intentionally excludes `.git` and
its generated `build/` directory; a post-copy comparison after those exclusions was empty. The
archive itself contains only candidate task source, tests, requirements, and architecture documents.

## Initial result

**Build/API gate: PASS.** Public headers are byte-identical to the supplied contract. The project
exposes `vwmini::vwmini`, uses C++23, and builds warning-clean with GCC 16.2.1 and Clang 22.1.8
under `-Wall -Wextra -Wpedantic -Werror`.

**Safety gate: PASS.** Candidate tests pass under Clang ASan/UBSan/float-cast-overflow without a
report. The observed defects are finite arithmetic and crowd-progress failures, not a crash,
sanitizer finding, or reproduced non-finite/out-of-mesh normative state; the automatic 40-point
safety cap does not apply at this stage.

**Conformance tracks:** G **15/15**, N **30/30**, S **32/33**, C **3/4**; total **80/82**.

| Track | Result | Initial evidence |
|---|---:|---|
| Geometry | 15/15 | All vector and triangulation cases pass. |
| Navmesh/path | 30/30 | All topology, tolerance, direct/bent path, determinism, and containment cases pass. |
| Simulation | 32/33 | The far-origin stored float endpoint can exceed its speed budget by one ULP. |
| Crowd | 3/4 | Crossing, overtaking, and overlap recovery pass; close reflex-corner following stalls. |

The candidate's own suite is unusually substantial: **100** individually discovered GTest cases
across eight focused binaries, all passing in GCC Debug and Release, Clang Debug, and Clang
ASan/UBSan/float-cast-overflow builds. It also supplies a checked `.clang-format`, architecture
and implementation-plan documents, private double-precision geometry helpers, and no added runtime
framework or global mutable state.

## Material defects and repair scope

1. **SIM-008/SIM-012: float storage can exceed the speed budget.** At `{10000,0}`, speed `1`,
   and `step(0.0006f)`, the committed position changes by `0.0009765625`, greater than the
   `0.000600000028` allowance. `src/vwmini/simulation.cpp` narrows a legal double position to a
   float without testing the displacement actually represented by that float.
2. **SIM-010/SIM-011/SIM-012: feasible close following stalls at a reflex corner.** In a
   one-metre L passage, the leader reaches while the nearby follower remains `Moving` with zero
   velocity after the 10-second budget. The local snapshot candidate policy preserves separation
   but has no retained progress/passing choice once the terminal agent occupies the arm.
3. **MSH-001: extreme finite vector results violate the public finite-result guarantee.**
   `length({FLT_MAX,FLT_MAX})` returns infinity; `normalized` then returns `{0,0}` for a non-zero
   vector. Scale-safe normalization and a finite representation policy for unrepresentable lengths
   are needed.
4. **NFR-003/CMake: tests are an unconditional GTest dependency.** Even with `BUILD_TESTING=OFF`,
   configure fails when GTest is unavailable because the root unconditionally adds `tests/` and it
   calls `find_package(GTest REQUIRED)`. A library-only consumer should configure without a test
   package.
5. **SIM-013 at identifier exhaustion.** The monotonic `uint32_t` id source wraps to `1`; if that
   id is live, insertion is ignored and `add_agent` returns an id that is not a live agent. This is
   a source-audited boundary case; it needs an exhaustion diagnostic rather than reuse.

The architecture documents are otherwise credible and unusually specific. Two material accuracy
issues remain: they promise both never-reused identifiers despite the wrap-to-1 implementation, and
substeps no larger than 1/60 second despite the explicit 4096-step cap yielding larger steps above
roughly 68.3 seconds. Those policies should be documented accurately or changed.

## Comparison with other initial rounds

On the same current 82-test suite, this is the strongest first-round public result evaluated so
far: **80/82**, compared with GPT-5.6 terra's **78/82** initial result. It does so with a much
broader candidate test suite (100 cases versus Terra's one custom executable), a more granular
module layout, real incremental Git history, checked formatting, and safe moved-from queries.

The prior local Qwen 3.8 27B initial round was evaluated against an earlier 72-test suite and
scored **69/72**. That is a strong historical result but is not numerically interchangeable with
this 80/82 current-revision result. Qwen Max's first-round strengths are engineering process,
coverage, geometry/navmesh completeness, and a clean lifecycle design; its remaining weakness is
not general implementation completeness but the last-mile float-storage and constrained
close-following policies. Time-to-completion is recorded as context only, not treated as a quality
score.

## Commands and toolchain

```sh
# GCC 16.2.1 Debug and Release, candidate tests
cmake -S solutions/qwen38-max-venice-run-01 -B /tmp/qwen38max-debug \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS='-Wall -Wextra -Wpedantic -Werror'
cmake --build /tmp/qwen38max-debug --parallel
ctest --test-dir /tmp/qwen38max-debug --output-on-failure
# Result: 100/100 passed. Equivalent Release result: 100/100 passed.

# Clang 22.1.8 and sanitizer evidence
CC=clang CXX=clang++ cmake -S solutions/qwen38-max-venice-run-01 \
  -B /tmp/qwen38max-clang -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-Wall -Wextra -Wpedantic -Werror'
cmake --build /tmp/qwen38max-clang --parallel
ctest --test-dir /tmp/qwen38max-clang --output-on-failure
# Result: 100/100 passed.
# Clang ASan/UBSan/float-cast-overflow: 100/100 passed; no report.
CC=clang CXX=clang++ cmake -S solutions/qwen38-max-venice-run-01 \
  -B /tmp/qwen38max-san -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-Wall -Wextra -Wpedantic -fsanitize=address,undefined,float-cast-overflow -fno-omit-frame-pointer' \
  -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined,float-cast-overflow'
cmake --build /tmp/qwen38max-san --parallel
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1 \
  ctest --test-dir /tmp/qwen38max-san --output-on-failure

./evaluator/conformance/run.sh solutions/qwen38-max-venice-run-01 /tmp/qwen38max-conformance
# Result: 80/82; G 15/15, N 30/30, S 32/33, C 3/4.

# Demonstrates the library-only configuration defect:
cmake -S solutions/qwen38-max-venice-run-01 -B /tmp/qwen38max-no-gtest \
  -DBUILD_TESTING=OFF -DCMAKE_DISABLE_FIND_PACKAGE_GTest=ON
# Result: configure fails at tests/CMakeLists.txt: find_package(GTest REQUIRED).
```

## Repair request

The focused repair is recorded in
[`repair-prompts/qwen38-max-venice-run-01-repair-01.md`](repair-prompts/qwen38-max-venice-run-01-repair-01.md).
It is intentionally limited to finite vector/storage correctness, close-following progress,
library-only configuration/identifier exhaustion, and matching documentation/tests. Final scoring
is deferred until that repair is evaluated.
