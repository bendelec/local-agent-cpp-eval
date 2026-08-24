# Evaluator Rubric

This document is private. Score the submitted candidate source, not its prose or
class count. A simple direct design is preferred when it is correct and understandable.

## Gates

A submission that does not configure/build with CMake and C++23, does not expose the
fixed public API, or has a demonstrated core safety failure cannot score above **40/100**.
Core safety failures are sanitizer-reported undefined behavior or memory errors on valid
input, a crash, or reproducible non-finite/out-of-mesh state in normative conformance.
Other valid-input numerical robustness defects remain substantial functional deductions but
do not automatically invoke the cap. Do not award points for unrequested capabilities.

## Score (100 points)

| Area | Points | What earns full credit |
|---|---:|---|
| Geometry and mesh validation | 20 | Correct finite/simple/CCW checks, deterministic triangulation, transactional triangle-mesh creation, robust shared-edge topology and containment tolerance. |
| Pathfinding | 20 | Correct direct/disconnected/bent paths; deterministic tie handling; safe contained segments; meaningful portal/funnel shortening rather than centre-only routing. |
| Agent lifecycle and stepping | 20 | Exact public error/state semantics, safe substepping, speed/arrival behavior, route refresh and removal handling. |
| Local crowd behavior | 15 | Snapshot-based deterministic local avoidance that passes the private crossing/overtaking fixtures without tunnelling or non-finite state. |
| Tests and functional discipline | 10 | Focused deterministic tests covering successful and error paths; no weakening or test-only hardcoding. |
| Architecture | 10 | Cohesive responsibilities, low coupling, sensible inward dependencies, understandable ownership; no needless framework or leaked mutable internals. |
| C++ quality | 5 | RAII/value semantics, const correctness, meaningful names, small comprehensible functions, warning-clean code, no debug artifacts. |

## Architecture review questions

Do **not** require a particular file layout or algorithm. Instead ask:

1. Can geometry predicates and validation be understood without simulation state?
2. Is navmesh topology/pathfinding isolated from agent lifetime and steering policy?
3. Does simulation own its mutable agent state clearly, without global state or raw
   ownership?
4. Are public contracts implemented directly rather than buried behind needless virtual
   abstractions, factories, or speculative extension layers?
5. Is duplicated numeric/geometry logic consolidated where it represents one shared
   rule, without creating a god utility or abstracting clear local code prematurely?
6. Would a maintainer be able to alter avoidance without destabilising mesh validation?

## Deductions

- Deduct functional points, not architecture points alone, for incorrect behavior.
- Deduct up to 5 architecture/quality points for each material scope violation that
  increases complexity (threads, third-party frameworks, hidden global state, etc.).
- Do not penalize different valid algorithms, private data layouts, or extra focused
  tests. Do penalize hard-coded private fixture coordinates or behavior keyed to tests.
- Implementation NCLOC and optional complexity metrics are supplemental review evidence,
  not score targets. Prefer fewer implementation lines only when correctness, safety,
  tests, and readability are otherwise equivalent; do not reward metric gaming.
