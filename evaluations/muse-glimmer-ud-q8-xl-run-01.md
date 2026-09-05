# Evaluation — Muse Glimmer (UD-Q8_XL run)

## Run identity

| Field | Value |
|---|---|
| Model | Muse Glimmer |
| Runtime | Local Lemonade; UD-Q8_K_XL. Exact backend/version is not retained. |
| Run label | `muse-glimmer-ud-q8-xl-run-01` |
| Task revision | Current VWmini task, including NFR-009 and NFR-010; conformance revision 2 |
| Final source | [`../solutions/muse-glimmer-ud-q8-xl-run-01-repair-02/`](../solutions/muse-glimmer-ud-q8-xl-run-01-repair-02/) |
| Final tree fingerprint | `4247eab34dcad49944a55c657705cbcf2850db15a36aa57fe1c269bf82048b5e` |
| Repair limit | Two repair prompts |
| Initial result | **35 / 100** |
| Final result | **36 / 100** |

The initial model report and two repair reports were delivered in chat rather than written
into the model workspace. They are retained as operator-supplied evidence in
[`../sessions/muse-glimmer-ud-q8-xl-run-01/`](../sessions/muse-glimmer-ud-q8-xl-run-01/).
The immutable final source archive above is the evaluation subject.

## Efficiency and session context (operator report; not scoring)

At finalization, the operator supplied the following whole-run aggregates, spanning the
initial pass and both repair cycles. A *turn* means one model response; a *round* means one
initial or repair prompt cycle. *Input* and *output* are the operator's recorded service
request and response token totals. The detailed accounting scope (for example, system, tool,
and repeated-context tokens) is defined in the sister project's
[reflective-context results (pinned predecessor record)](https://github.com/bendelec/reflective-pi/blob/3283c475683d2b6861f13381339f8764efda203a/packages/evals/reflective-context-results.md).
That record retains the completed predecessor runs' session telemetry and its mechanical
accounting protocol. Muse's corresponding result is still being prepared there; until it is
uploaded, this report preserves the operator-provided Muse aggregate as provisional context.
The provisional Muse row is not yet protocol-comparable to the mechanically reconstructed
predecessor rows. VWmini itself retains no raw session exports, as described by the
[session-retention policy](../sessions/README.md). Token volume, wall time, and session
behavior are not VWmini rubric criteria and do **not** change the score.

The table is a selected set of operator-identified comparators, not a complete token
comparison or a leaderboard.

| Run | Turns | Input tokens | Output tokens |
|---|---:|---:|---:|
| Muse Glimmer (`muse-glimmer-ud-q8-xl-run-01`) | 66 | 3.4M | 57k |
| Poolside Laguna S 2.1 DS4 (`laguna-s-2.1-ds4-run-01`) | 411 | 20.6M | 281k |
| Qwen 3.8 27B (`qwen38-27b-q8-xl-run-01`) | 806 | 51.2M | 961k |

On these operator-provided aggregates, Muse's input and output token volume is more than
an order of magnitude below Qwen's. These are provider/runtime token units with different
possible tokenization and accounting scope; they describe recorded volume, not comparable
compute, cost, or a measured efficiency ratio. The operator further reports that Muse and
Qwen used the same hardware with comparable serving throughput, and that each corresponding
initial or repair cycle for Muse took less than half Qwen's wall time. The
[run-control policy](../README.md#run-control-policy) requested high thinking/reasoning, or
the nearest semantic equivalent, for every published run. Muse was therefore not
intentionally assigned a lower requested setting than Qwen. These operator observations
provide context for the unusually terse run, but do not explain or excuse the final
engineering defects.

The operator considers it plausible—but untested—that closer feedback loops, more granular
repair prompts, and earlier explicit guidance to split the implementation might have enabled
more progress within comparable wall time while retaining lower token volume. This compact
operator hypothesis is recorded at the operator's request; it is neither a VWmini quality
adjustment nor evidence that the final archive is more correct.

For neutral run-accounting context, the operator distinguishes an *output-limit event* (a
service response ending at its configured output allowance), *automatic compaction* (the
runner reaching its context-management trigger), and a *proactive prune* (manual removal of
no-longer-needed context before that trigger). The comparison population is the six completed
runs in the [overview](overview.md): both DeepSeek V4 Flash runs, Qwen, both Poolside Laguna
runs, and Muse. The operator reports that, across its whole run, Muse had neither an
output-limit event nor automatic compaction; it made one proactive prune at approximately
70% of context capacity during the first repair and needed none in the initial pass. The
operator also reports that each other run had at least one output-limit event or automatic
compaction during its initial pass. The linked external record retains the predecessor event
telemetry; for example, it records Qwen's six automatic compactions. Muse's corresponding
external entry is pending. These facts are run-accounting context only, not a session-quality
assessment.

## Repair outcome

The first repair improved public conformance from **73 / 77** to **75 / 77** by correcting
the previously failing navmesh/path cases and the ordinary overtaking crowd case. It did not
correct diagonal crossing progress and regressed the initial-overlap recovery case that the
initial archive had passed. The second repair did not make a functional implementation change:
formatting `repair-01/src/vwmini.cpp` with the available `clang-format` produces a file
byte-identical to final `src/vwmini.cpp`; `CMakeLists.txt`, the test file, and both
architecture documents are byte-identical between the two repair archives. Thus the final
result remains **75 / 77**:

| Track | Result | Material failures |
|---|---:|---|
| Geometry | 15 / 15 | — |
| Navmesh/path | 29 / 29 | — |
| Simulation lifecycle | 29 / 29 | — |
| Crowd | 2 / 4 | diagonal crossing agents remain `Moving`; initially overlapping agents do not materially separate |

The preserved repair-02 report claims all work packages are complete and verified, snapshot
avoidance, active Release tests, and clean formatting. Those claims are not supported by the
submitted archive or independent checks below.

## Final verification

The final archive builds as C++23 in GCC 16.2 and Clang 22.1.8 Debug configurations with
`-Wall -Wextra -Wpedantic -Werror`; its one native executable reports 1/1 passing under both.
A GCC Release build reports 1/1 passing, and a GCC ASan/UBSan Debug native run passes without
a report. `BUILD_TESTING=OFF` builds the library and has no test target. Supplied public
headers are byte-identical to the candidate package.

These are limited positives. The native test is still an assert-only executable, so its
assertions disappear under `NDEBUG`; the observed Release output is simply its unconditional
success prints. Production `src/vwmini.cpp` is now clean under `clang-format --dry-run
--Werror`, but the unchanged `tests/test_basic.cpp` produces **1,008** format diagnostics.
The repair report only ran the formatter on production source and did not perform the
promised Release-effective test conversion or full source/test formatting check.

Normative conformance was run against the final immutable archive. Geometry, navmesh/path,
and lifecycle all pass. Crowd crossing fails only at the final arrival assertion: both agents
remain `Moving` after the required twelve simulated seconds. Initially overlapping agents
reach a greatest centre separation of only approximately **0.10000 m**, below the required
**0.25 m** observable recovery threshold. Overtaking and close reflex-corner following pass.

## Reproduced findings beyond the public suite

### High — avoidance is neither simultaneous nor live in feasible crossing

`Simulation::step` selects agent `i` while reading `chosen[j]`
(`src/vwmini.cpp:741-807`). For later agents that value is its default zero velocity; for
earlier agents it is a decision already affected by ordering. It is therefore not a decision
from one immutable snapshot, despite the report and architecture claims. Its only conflict
response is stopping, with no tangential or separation candidate. This explains both failed
crowd cases: crossing preserves separation by mutual blocking but never reaches either goal,
while an initially overlapping pair preserves its overlap rather than attempting recovery.

### High — huge valid duration still does not return after a crowd stalls

The loop stores `remaining` as `float`, subtracts an at-most `0.05f` substep, and has no
no-progress termination policy (`src/vwmini.cpp:700-867`). After two opposing agents block
each other, a call to `step(std::numeric_limits<float>::max())` did not return before a
two-second timeout (status 124). The duration is accepted finite input. This contradicts the
architecture claim that huge durations terminate safely; the passing native huge-duration
case has only one agent, which reaches its goal before this failure mode arises.

### High — finite extreme-coordinate meshes are not supported safely

Topology matching quantizes every coordinate by `epsilon` to `long long`
(`src/vwmini.cpp:104-130`). Much of the accepted finite-float range cannot be represented
that way. A valid CCW triangle using coordinates near `FLT_MAX` is rejected with
`InvalidArgument`, although all vertices are finite and its signed double area is positive
and enormous. The same probe shows `length({FLT_MAX, FLT_MAX}) == infinity` and
`normalized({FLT_MAX, FLT_MAX}) == {0,0}`, violating the supplied header's finite-input
length contract and usual nonzero-vector normalization semantics. The native sanitizer suite
does not exercise either accepted-input case.

### Medium — mesh validation accepts degenerate and duplicate overlapping input

The triangle check compares *twice* signed area with `epsilon²`
(`src/vwmini.cpp:257-261`), accepting a triangle with signed area `7.5e-9`, below the
specified strict `1e-8` threshold. `triangulate_simple_polygon` also accepts the same
degenerate outline. `NavMesh::create({triangle, triangle})` succeeds, although duplicate
triangles have overlapping interiors and must be rejected. Strict vertex-in-triangle tests
miss coincident boundaries, while shared endpoint tests skip the remaining edge-intersection
checks (`:257-315`).

### Medium — tests and documents do not describe the delivered evidence

All test checks remain `assert` calls. The epsilon edge test explicitly treats rejection of
its stated valid case as acceptable; the huge-duration test requires neither timely return
under a blocked crowd nor progress; and the avoidance test checks separation only, so a
deadlock passes. No test was added in the second repair.

The architecture still says `NavMesh::Impl` holds adjacency and a vertex-deduplication map;
it actually holds triangles and boundary edges, while `find_path` rebuilds adjacency. It also
claims candidate velocity selection with snapshot semantics and safe huge-duration stepping,
which the source does not implement. The plan marks all work packages and verification done.
This is a material NFR-009/NFR-010 mismatch, not a cosmetic stale comment.

## Maintainability evidence

The final source is **835** physical NCLOC in one translation unit covering numeric policy,
triangulation, mesh validation, topology, pathfinding, mutable agent state, and avoidance.
Clang CFG analysis of `Simulation::step` reports 95 basic blocks and 138 edges, for McCabe
complexity **45**. That gives the decomposition category a 13-point rubric ceiling before
qualitative review. The coupled decision/integration/route-state loop is directly associated
with the crowd and huge-duration defects. Positive choices include C++23 value ownership,
private pImpl state, immutable mesh sharing, no third-party runtime dependency, and
warning-clean compiler builds.

## Score

No automatic 40-point safety cap applies: normative conformance did not reproduce a crash,
sanitizer report, or non-finite/out-of-mesh state. The valid-input nontermination and
finite-scale numerical failures remain substantial functional defects.

| Area | Score | Assessment |
|---|---:|---|
| Architecture and dependency design | 6 / 20 | Ownership is sensible, but one translation unit couples all policies and the required architecture record describes storage and stepping behavior that do not exist. |
| Decomposition and complexity | 5 / 20 | The CFG-45 result sets a 13-point ceiling; the 835-NCLOC god translation unit, unused topology work, and policy-dense `step` justify a materially lower score. |
| C++ clarity and discipline | 5 / 10 | RAII, values, compiler warning cleanliness, and formatted production source are positives. Test formatting remains broken, and valid finite geometry/vector inputs have severe numeric behavior. |
| Tests and functional discipline | 1 / 15 | One assert-only smoke executable supplies no effective Release checks and misses every remaining crowd, stalled-duration, extreme-scale, overlap, and degeneracy defect. |
| Functional conformance beyond the gate | 19 / 35 | 75/77 is meaningful progress over the initial submission, but the crossing and overlap-recovery crowd cases, valid stalled huge-duration termination, finite extreme-scale geometry, and mesh validity probes remain material failures. |
| **Final total** | **36 / 100** | |

## Commands used

```sh
# Normative conformance
evaluator/conformance/run.sh \
  solutions/muse-glimmer-ud-q8-xl-run-01-repair-02 \
  /tmp/muse-glimmer-r2-conformance

# Native warning-clean builds and tests
CC=gcc CXX=g++ cmake -S solutions/muse-glimmer-ud-q8-xl-run-01-repair-02 \
  -B /tmp/muse-glimmer-r2-g++ -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-Wall -Wextra -Wpedantic -Werror'
cmake --build /tmp/muse-glimmer-r2-g++ --parallel
ctest --test-dir /tmp/muse-glimmer-r2-g++ --output-on-failure
CC=clang CXX=clang++ cmake -S solutions/muse-glimmer-ud-q8-xl-run-01-repair-02 \
  -B /tmp/muse-glimmer-r2-clang -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-Wall -Wextra -Wpedantic -Werror'
cmake --build /tmp/muse-glimmer-r2-clang --parallel
ctest --test-dir /tmp/muse-glimmer-r2-clang --output-on-failure

# Sanitizer, Release, and library-only configurations
CC=gcc CXX=g++ cmake -S solutions/muse-glimmer-ud-q8-xl-run-01-repair-02 \
  -B /tmp/muse-glimmer-r2-san-gcc-final -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
  -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined'
cmake --build /tmp/muse-glimmer-r2-san-gcc-final --parallel
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1 \
  ctest --test-dir /tmp/muse-glimmer-r2-san-gcc-final --output-on-failure
CC=gcc CXX=g++ cmake -S solutions/muse-glimmer-ud-q8-xl-run-01-repair-02 \
  -B /tmp/muse-glimmer-r2-release-gcc-final -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/muse-glimmer-r2-release-gcc-final --parallel
ctest --test-dir /tmp/muse-glimmer-r2-release-gcc-final --output-on-failure
cmake -S solutions/muse-glimmer-ud-q8-xl-run-01-repair-02 \
  -B /tmp/muse-glimmer-r2-notest -DBUILD_TESTING=OFF
cmake --build /tmp/muse-glimmer-r2-notest --parallel

# Archive-only formatting check
clang-format --dry-run --Werror \
  solutions/muse-glimmer-ud-q8-xl-run-01-repair-02/src/vwmini.cpp \
  solutions/muse-glimmer-ud-q8-xl-run-01-repair-02/tests/test_basic.cpp
```

## Final assessment

Muse Glimmer was exceptionally quick to complete both repair cycles. The first repair did
fix two path/mesh conformance failures; the second was a production-source formatting pass
rather than the requested avoidance/test/documentation repair. The final archive is a
compilable, partially conforming implementation with sensible basic ownership, but it is not
a dependable navigation/crowd library: feasible crossing agents can deadlock, overlapping
agents do not recover, a valid finite duration can fail to return, and accepted finite numeric
input is not robust at scale.
