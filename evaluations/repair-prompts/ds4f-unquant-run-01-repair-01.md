# VWmini Repair Pass

Continue work in the current VWmini workspace. Preserve the public API and all
existing requirements. Diagnose and repair the following general behaviors; do
not hard-code a particular test coordinate, agent id, or mesh layout.

## 1. Tolerance-contained direct paths

A segment whose endpoints lie within the documented mesh containment tolerance
must be handled consistently with `NavMesh::contains`. A direct route across a
square can begin and end slightly outside its mathematical boundary but within
the accepted epsilon band; `find_path` must return the exact supplied endpoints
when the segment is contained under that same policy, rather than returning
`NoPath` because an initial boundary event is misclassified.

Use one coherent segment-containment rule for direct-path selection and final
route verification. Preserve exact caller endpoints and deterministic output.

## 2. Mesh topology and numerical robustness

Audit shared-edge matching and invalid topology detection. Mesh adjacency must
respect the documented endpoint tolerance, and a vertex lying on another
triangle's open edge must be rejected as an invalid T-junction rather than
silently accepted as a crack. Keep valid complete shared edges connected.

Also ensure valid finite public inputs do not cause invalid integer conversions
or non-finite simulation state. In particular, `step(float)` accepts every
finite non-negative duration: very large finite durations must not overflow a
substep-count conversion or require an impractical unbounded loop. Preserve
finite, contained state and speed bounds. Use numerically safe intermediate
arithmetic where needed.

## 3. Local crowd progress and separation

The current local avoidance can permit a faster same-lane agent to overlap the
slower agent it overtakes. It can also leave two initially non-overlapping,
same-direction agents permanently stuck when they pass through a valid
L-shaped reflex corner. Repair the general steering/containment interaction so
that ordinary crossing and overtaking remain collision-free and reach their
distinct goals, and close-following agents can make progress through a
connected reflex-corner route.

Do not disable avoidance near corners, special-case a fixture, or treat an
identical terminal goal as a required crowd-solver problem. Preserve
simultaneous snapshot decisions, determinism, containment, and speed limits.

## Regression coverage and verification

Add focused deterministic native regressions for each repaired behavior:

- an epsilon-contained direct path preserving exact endpoints;
- T-junction rejection and tolerance-matched complete shared-edge adjacency;
- a large finite `step` duration that leaves every state finite and contained;
- overtaking with per-step separation checks; and
- close following through an L-shaped reflex corner, with distinct feasible
goals, per-step finite/contained/speed/separation checks, and both agents
reaching.

Retain existing tests. Build from scratch with GCC and Clang under
`-Wall -Wextra -Wpedantic`, run CTest, and run an ASan/UBSan build if available.
Update the architecture document, implementation plan, and completion report
to describe the actual repair and verification. Keep the design direct; do not
add framework code, global state, or an unnecessary crowd-planning layer.
