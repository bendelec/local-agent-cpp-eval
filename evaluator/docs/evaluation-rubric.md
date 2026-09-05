# Evaluator Rubric

This is published evaluator guidance; it is not supplied in a candidate task package.
Score the submitted candidate source, not its prose or class count. A simple direct
design is preferred when it is correct and understandable.

## Gates

A submission that does not configure/build with CMake and C++23, does not expose the
fixed public API, or has a demonstrated core safety failure cannot score above **40/100**.
Core safety failures are sanitizer-reported undefined behavior or memory errors on valid
input, a crash, or reproducible non-finite/out-of-mesh state in normative conformance.
Other valid-input numerical robustness defects remain substantial functional deductions but
do not automatically invoke the cap. Do not award points for unrequested capabilities.

## Score (100 points)

Functional correctness is a prerequisite for a credible implementation, not a substitute
for maintainable design. Build/API/safety gates establish the minimum bar. The score then
weights architecture and maintainability (**50 points**) more heavily than functional
conformance beyond that bar (**35 points**).

| Area | Points | What earns full credit |
|---|---:|---|
| Architecture and dependency design | 20 | Cohesive responsibilities, explicit stable seams, low coupling, sensible inward dependencies, understandable ownership, and no leaked mutable internals. |
| Decomposition and complexity | 20 | Small coherent functions and modules; no unnecessary bespoke machinery or duplicated policy; control flow and nesting proportionate to the task. |
| C++ clarity and discipline | 10 | RAII/value semantics, const correctness, meaningful names, standard-library use where appropriate, warning-clean code, and no debug artifacts. |
| Tests and functional discipline | 15 | Focused deterministic tests with independent oracles where practical; successful and error paths; no weakening, fixture hard-coding, or shared-bug test oracle. |
| Functional conformance beyond the gate | 35 | Correct geometry, paths, lifecycle, and crowd behavior across normative and targeted reviewer probes, with deterministic finite contained state. |

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
- Implementation NCLOC and complexity metrics are evidence for decomposition review, not
  standalone score targets. Do not reward line-count gaming.
- Measure source-only non-comment LOC (NCLOC) consistently. Use Clang CFG McCabe
  complexity (`E - N + 2`) and maximum lexical brace nesting as objective per-function
  signals, then combine them with source review for missed generalization, duplicated
  bespoke policy, coupling, and unclear responsibility boundaries.
- For the 20-point decomposition/complexity category, the worse maximum CFG/nesting pair
  establishes a ceiling before subjective review: <=20/<=5 -> 20; 21-35/6 -> 17;
  36-50/7 -> 13; 51-70/8-9 -> 8; >70/>=10 -> 3. Source-review findings may lower the
  score within that ceiling. These thresholds are maintainability evidence, not a
  correctness gate.
