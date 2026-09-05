# Repair Follow-up

Continue in the current VWmini workspace. Preserve the supplied public API, documented
requirements, and behavior that already works. Do not replace files wholesale, weaken or
delete regressions, hard-code an agent id, coordinate, route, or fixture-specific steering
rule, or add frameworks, third-party dependencies, global mutable state, or an unrequested
global crowd planner.

The prior repair materially improved geometry, mesh, paths, lifecycle behavior, CMake
configuration, and ordinary overtaking/corner-following behavior. Independent verification
now passes geometry, navmesh/path, and simulation-lifecycle checks, but two local-avoidance
contract defects and engineering-evidence defects remain. Fix their policies rather than
only their examples.

## 1. Make local avoidance genuinely simultaneous, collision-free when feasible, and live

The current velocity loop claims snapshot decisions, but it reads `chosen[j]` while selecting
agent `i`. It therefore lets processing order determine the predicted motion and does not
select a mutually consistent set of velocities from one immutable snapshot. In a feasible
two-agent diagonal crossing, both agents preserve separation but then remain `Moving` rather
than reaching their goals. Stopping on a prospective collision is not a sufficient policy if
it creates permanent mutual blocking.

At every internal substep, derive all candidate motion from the same pre-update positions,
velocities, routes, radii, and speeds; then make a deterministic bounded local choice that
is valid against the contemporaneous choices of the other agents. Candidate motion must
include progress and safe yielding/separating or tangential alternatives. A local solution
that preserves non-overlap and lets both agents progress in open space must not be discarded
solely because of insertion/order bias. Commit positions only after all decisions are made.

For initially overlapping/coincident discs, retain finite contained speed-bounded state and
make a deterministic material separation attempt when open space exists. The current pair
starting only `0.1` apart reaches only about `0.1` separation after four seconds; it is not
an observable recovery. This robustness case need not become a general crowd solver, but it
must not merely stall or preserve the overlap.

For normal, initially non-overlapping crossing and overtaking in an open convex cell,
separation after **every returned `step`** must be at least `radius_a + radius_b - 1e-3f`,
and both agents must reach their valid distinct goals. Preserve successful overtaking and
close-following-around-a-corner behavior while fixing the crossing deadlock. Do not solve
this by teleporting, exceeding speed bounds, moving outside the mesh, changing goals, or
special-casing the checked layouts.

Add deterministic tests with checks that remain active in Release for:

- a non-collinear open-space crossing through the moment of closest approach, checking each
  returned step for finite, contained, speed-bounded, non-overlapping states and eventual
  arrival of both agents;
- an initially overlapping open-space pair with distinct goals, checking a material
  separation increase as well as finite, contained, speed-bounded motion;
- the already working overtaking and bent/reflex-corner following cases, so this correction
  does not regress them.

## 2. Make the test and formatting evidence real

The submitted test executable still consists entirely of `assert` checks. Those checks are
compiled out under `NDEBUG`, so its Release result does not validate the claimed policies.
Replace them with a small deterministic test harness whose failures remain effective in
Debug and Release, or another dependency-free equivalent. Cover successful and rejected
behavior relevant to the repaired policy; do not print success while an individual check has
failed.

`clang-format --dry-run --Werror` reports thousands of diagnostics in the implementation and
test source. Format the source and tests you own with the available formatter and leave no
format diagnostics for those files. The supplied public headers are fixed and must remain
byte-for-byte unchanged; do not reformat them merely to improve this check. If you add a
format configuration, keep it small and apply it consistently to new/private source and
tests.

## 3. Correct the engineering description instead of claiming nonexistent design

The architecture document says that `Impl` stores adjacency and a vertex-deduplication map,
but the delivered `Impl` stores only triangles and boundary edges and reconstructs adjacency
inside `find_path`. It also says the step policy uses candidate velocity selection, yet the
current sequential loop does not meet its stated snapshot semantics. The implementation plan
marks every slice done and its tests/formatting verification complete even though the source
still has the defects above.

After the code and tests are correct, update the architecture and implementation plan to
describe the submitted storage, dependencies, avoidance decision/commit phases, test design,
and actual verification. You may make a cohesive local split if it genuinely clarifies the
geometry/mesh/simulation boundary, but a broad rewrite is neither required nor wanted.

Before completion, run and report only actual outcomes for: native Debug and Release tests;
GCC and Clang warning-clean builds with `-Wall -Wextra -Wpedantic`; an ASan/UBSan test build
if available; a `BUILD_TESTING=OFF` library-only build; and a formatting check limited to
the source/test files you own. Keep invalid duration validation transactional and retain the
already repaired finite-scale geometry, epsilon shared-edge matching, continuous route
containment, and huge finite-duration termination behavior.
