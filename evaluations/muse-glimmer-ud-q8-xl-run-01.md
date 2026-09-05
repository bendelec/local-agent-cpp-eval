# Evaluation — Muse Glimmer (UD-Q8_XL initial run)

## Run identity

| Field | Value |
|---|---|
| Model | Muse Glimmer |
| Runtime | Local Lemonade; UD-Q8_K_XL. Exact backend/version is not retained. |
| Run label | `muse-glimmer-ud-q8-xl-run-01` |
| Task revision | Current VWmini task, including NFR-009 and NFR-010; conformance revision 2 |
| Evaluated source | [`../solutions/muse-glimmer-ud-q8-xl-run-01/`](../solutions/muse-glimmer-ud-q8-xl-run-01/) |
| Source tree fingerprint | `f9c8fe14078dbbcf74a551356cc4b3fc22cdb6e02aaac88897eae6ad27992218` |
| Model completion report | [operator-supplied chat report](../sessions/muse-glimmer-ud-q8-xl-run-01/model-completion-report.md) |
| Initial result | **35 / 100** |

The completion report was delivered in chat rather than written into the model workspace.
The implementation prompt required a completion report but did not prescribe a file path,
so that delivery is compliant. The report is preserved separately as operator-supplied run
evidence; the immutable archived source tree above is the evaluation subject.

## Verification

The archive builds as C++23 and preserves all supplied public headers byte-for-byte. Its
single native CTest executable passes in Debug under GCC 16.2 and Clang 22.1.8. A GCC
ASan/UBSan Debug build also passes that native test, and both compilers accept the source
with `-Wall -Wextra -Wpedantic -Werror`.

The native test evidence is extremely limited: it is one 57-line `assert`-based smoke
test. A Release build still reports 1/1 passing, but `NDEBUG` removes its assertions and
leaves only `All basic tests passed` output. `BUILD_TESTING=OFF` is ignored and still
builds the test executable. `clang-format --dry-run --Werror` reports 1,311 formatting
diagnostics across the source and test files.

Normative public-API conformance is **73 / 77**:

| Track | Result | Material failures |
|---|---:|---|
| Geometry | 15 / 15 | — |
| Navmesh/path | 27 / 29 | finite extreme-coordinate containment fails; a boundary-tolerance direct path returns four points rather than exactly `[start, goal]` |
| Simulation lifecycle | 29 / 29 | — |
| Crowd | 2 / 4 | both crossing and overtaking permit material disc overlap |

The crowd crossing reaches a minimum centre separation of approximately **0.00455** where
the required minimum is **0.499**. Overtaking reaches approximately **0.48360**. These are
not tolerance-scale misses; they violate the normal feasible open-space behavior required
by SIM-011.

## Reproduced findings

### High — a valid finite duration does not return

`Simulation::step` repeatedly subtracts a fixed `0.05f` from the supplied duration
(`src/vwmini.cpp:617-621`). At `FLT_MAX`, that subtraction makes no representable progress,
so a valid call never terminates. A public-API probe with one moving agent and
`step(std::numeric_limits<float>::max())` remained running until a two-second timeout
(status 124). The implementation must use a numerically safe duration/chunk policy; it
must not reject the accepted finite duration, silently discard elapsed time, or require an
unbounded number of substeps.

### High — local avoidance does not enforce collision-free feasible motion

Velocity selection (`:640-651`) adds a weak pairwise repulsion to the desired velocity,
then independently clamps each agent. The later sequential integration (`:654-696`) does
not validate predicted pair separation or repair a failed choice. Consequently, crossing
agents pass through one another almost completely and overtaking agents overlap. This
contradicts both the completion report's claim that the method is sufficient for the
open-cell acceptance criterion and SIM-010/011.

### High — complete edges are matched exactly rather than within epsilon

Both mesh creation (`:222-235`) and path adjacency construction (`:357-379`) key edges by
exact `Vec2` equality. MSH-005 instead requires corresponding complete edge endpoints no
farther than epsilon apart. A four-triangle probe with the two copies of a shared edge
separated by `5e-5f` is accepted and connected by the reference implementation, but this
archive rejects it as a `T-junction`. This also duplicates the same topology policy in two
places.

### Medium — path containment/directness and mesh predicates are numerically incomplete

`find_path`'s containment helper combines boundary intersections with 64 samples
(`:312-334`), which cannot establish SIM-002's requirement for every real point of a
segment. The boundary-tolerance fixture exposes an observable result: a contained direct
route returns four points rather than the required exact two-point path. Separately,
float cross products and squared lengths overflow for accepted finite extreme-scale mesh
coordinates, so an interior point is reported outside.

### Medium — source, tests, and design documentation are not maintainable evidence

All geometry, mesh validation, topology, pathfinding, simulation state, and avoidance
policy reside in one 721-line translation unit (684 source NCLOC). Clang CFG analysis
reports a maximum McCabe complexity of 41 in `find_path`, with further dense routines for
triangulation (30), mesh creation (28), and stepping (24). The rubric therefore limits
the decomposition category to 13 before source review; the monolithic, duplicated-policy
implementation warrants a materially lower score.

The architecture document claims that `Impl` holds adjacency and a vertex-deduplication
map, and that stepping uses waypoint-arrival-time substeps. The submitted `Impl` holds
only triangles and boundary edges, while stepping uses fixed 0.05-second chunks. The plan
claims verification for validation, containment, determinism, lifecycle, and crowd
behavior that its one smoke test does not perform. The model's chat report similarly says
the architecture/plan are current and the avoidance meets the open-cell criterion; both
claims are contradicted by the submitted source and public-API results.

## Score

| Area | Score | Assessment |
|---|---:|---|
| Architecture and dependency design | 7 / 20 | Value ownership and private pImpl storage are sensible, but one translation unit couples every subsystem and duplicates topology policy. The architecture description also misstates the delivered storage and stepping design. |
| Decomposition and complexity | 5 / 20 | The CFG-41 maximum establishes a 13-point ceiling. Geometry, mesh validation, pathfinding, agent lifecycle, and avoidance remain in one 684-NCLOC implementation file, with several independently complex routines and no coherent internal modules. |
| C++ clarity and discipline | 4 / 10 | C++23 values, `expected`, RAII, and warning-clean compiler builds are positives. Dense abbreviated code, unchecked float overflow, the ineffective containment line `ag.pos = ag.pos`, and 1,311 format diagnostics substantially reduce clarity and discipline. |
| Tests and functional discipline | 1 / 15 | One smoke executable supplies no focused coverage of errors, topology, paths, transitions, numerical edges, or crowds; its assertions vanish in Release. Reported/documented verification substantially overstates the delivered test evidence. |
| Functional conformance beyond the gate | 18 / 35 | 73/77 provides a real partial implementation, but finite-scale containment, direct boundary routing, both normative feasible crowd cases, epsilon edge matching, and valid huge-duration termination are material contract failures. |
| **Initial total** | **35 / 100** | |

The CMake/API gate passes. The sanitizer run on the insufficient native smoke test did not
report a memory or undefined-behavior error, so the rubric's narrow automatic 40-point
safety cap is not invoked. The valid-input nontermination remains a major functional and
robustness defect.

## Repair priorities

A repair should first make large finite duration handling terminating and speed-bounded,
centralize scale-safe geometry/topology/continuous-segment predicates, and derive
adjacency through epsilon-matched complete edges. It should then replace the repulsion
heuristic with simultaneous, predicted-separation velocity choices that preserve normal
crossing and overtaking. Split the single implementation unit along geometry, mesh/path,
and simulation responsibilities; add deterministic regressions for every repaired
failure, retain them in non-`NDEBUG`-dependent tests, format the source, and correct the
design/plan claims only after the implementation is verified.
