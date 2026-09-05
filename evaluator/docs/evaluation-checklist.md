# Candidate Evaluation Checklist

> **Evaluator procedure.** This published guidance is not supplied in the candidate
> task package. Use it with `evaluation-plan.md` and `evaluation-rubric.md`. It promotes
> consistent review; it does not authorize changing the candidate's public contract or
> rewarding unrequested features.

Record evidence and scores per track. Geometry failure must not erase separately earned
navmesh, lifecycle, crowd, test, architecture, or C++-quality evidence.

## 1. Intake and hard gates

- [ ] Submission contains only implementation-owned source/tests and required
  `docs/architecture/architecture.md` plus `docs/architecture/implementation-plan.md`;
  no private evaluator/reference material was added.
- [ ] Public headers, target `vwmini::vwmini`, C++23 requirement, enum values, defaults,
  and signatures remain canonical.
- [ ] Fresh CMake configure/build succeeds with `-Wall -Wextra -Wpedantic`.
- [ ] Candidate tests configure and run deterministically.
- [ ] Invalid public input does not crash, terminate, create NaNs, or invoke UB.
- [ ] Motion never leaves the mesh in the supplied safety checks.

If any build/API/core-safety gate fails, apply the 40-point cap from the rubric, but run
any independently compilable tracks and record their results.

## 2. Functional tracks (evidence for the 35-point conformance category)

Run `evaluator/conformance/run.sh CANDIDATE_SOURCE_DIR BUILD_DIR` when possible. Keep
its four label outputs in the review record.

### G — Geometry

- [ ] `Vec2` arithmetic and zero normalization are correct and finite.
- [ ] Simple CCW/hole-free outline validation distinguishes invalid argument from invalid
  finite geometry.
- [ ] Triangulation is deterministic, CCW, non-overlapping, and area-preserving.
- [ ] Collinear permitted cases and self-intersection/duplicate/winding failures behave
  sensibly.
- [ ] Triangle mesh construction is transactional and rejects invalid topology.
- [ ] Containment follows the stated finite boundary tolerance.

### N — Navmesh and paths

- [ ] Literal triangle mesh topology derives whole-edge adjacency only; vertex contact is
  not an adjacency.
- [ ] Direct path is exactly `[start, goal]`; equal endpoints yield one point.
- [ ] Outside/non-finite/disconnected endpoints report the correct error.
- [ ] Returned segments remain in the mesh, preserve exact endpoints, and are deterministic.
- [ ] Bent routes avoid non-walkable space and make a meaningful corner/portal shortening
  rather than following triangle centres or arbitrary detours.

### S — Agent lifecycle and stepping

- [ ] Add/remove/query/id behavior and every specified error code are exact.
- [ ] Goal assignment, `NoPath`, `Idle`, `Moving`, `Reached`, and `clear_goal` transitions
  are coherent and observable through `AgentState`.
- [ ] Step validation is transactional; zero duration changes nothing.
- [ ] Large normal frame deltas do not overshoot waypoints/goals, exceed max speed, or
  produce non-finite state.
- [ ] Positions stay contained; arrival radius semantics are correct.

### C — Local crowd behavior

- [ ] Decisions are based on a per-substep snapshot, not agent-update order.
- [ ] Crossing and overtaking fixtures make progress and avoid material disc overlap.
- [ ] Avoidance is deterministic; it does not teleport, tunnel, exceed speed, or leave
  the mesh.
- [ ] Pre-existing overlaps remain finite and receive a reasonable separation attempt.
- [ ] Do not require ORCA, global deadlock resolution, or a particular passing side.

## 3. Candidate tests and functional discipline (15 points)

- [ ] Tests are fast, deterministic, readable, and run through CMake/CTest or an equally
  clear command.
- [ ] They cover success and failure paths across geometry, mesh/path, simulation, and
  crowd behavior—not only trivial happy paths.
- [ ] Assertions check observable contracts/properties instead of private layout or an
  unnecessarily exact implementation trajectory.
- [ ] No test-only switches, fixture-coordinate hardcoding, weakened assertions, or
  swallowed errors are present.
- [ ] Candidate tests would likely catch a plausible regression in each major area.

## 4. Architecture, decomposition, and source review (40 points)

Read `docs/architecture/architecture.md` and
`docs/architecture/implementation-plan.md` beside the final source. They are evidence
of design/process quality, not substitutes for working code or opportunities to demand
the reference design.

### Architecture and dependency design (20 points)

- [ ] The document has an inline diagram and identifies actual components,
  responsibilities, dependencies, ownership/data flow, seams/interfaces, and key
  trade-offs.
- [ ] The diagram/document agrees with the submitted source and build graph.
- [ ] The plan has ordered, dependency-aware work packages with small slices, concrete
  verification, and status; material deviations are explained rather than erased.
- [ ] Responsibilities are cohesive: geometry/numeric predicates, immutable mesh/path
  work, and mutable simulation concerns are understandable without needless coupling.
- [ ] Dependencies flow toward stable lower-level facilities; no mutable global world
  state or hidden cross-module backdoor exists.
- [ ] Seams serve present needs (validation boundary, immutable mesh query, path query,
  steering/integration boundary) rather than speculative factories, virtual hierarchies,
  or framework scaffolding.
- [ ] Ownership and lifetime are obvious, especially mesh sharing/copying, simulation
  mutation, paths, and agents.

### Decomposition and complexity (20 points)

- [ ] Functions and modules have focused, cohesive responsibilities; policy stages that
  need independent reasoning or tests are not buried in one orchestration routine.
- [ ] The source avoids duplicated bespoke machinery and accidental coupling between
  geometry, mesh/path, and mutable simulation policy.
- [ ] Measure the maximum Clang CFG McCabe complexity and lexical brace nesting when
  practical. Apply the ceiling in `evaluation-rubric.md`; use the metric together with
  source review rather than as a standalone defect.

### C++ clarity and discipline (10 points)

- [ ] RAII and value semantics are used appropriately; raw owning pointers and leaks are
  absent.
- [ ] Const queries are const; error handling follows the fixed `Result` contract.
- [ ] Names, functions, and data structures are proportional and understandable.
- [ ] Numeric tolerances/predicates are centralized enough to avoid semantic drift.
- [ ] Repeated domain logic and bespoke helpers have been reviewed explicitly: unify
  genuinely shared rules, but do not penalize clear local code or demand abstraction
  for its own sake. Record concrete duplication/unification findings.
- [ ] Source is warning-clean, consistently formatted, and contains no debug output,
  generated artefacts, or dead abstraction layers.

### Supplemental source metrics

- [ ] Record implementation-only physical NCLOC with
  `./scripts/measure-source.py CANDIDATE_SOURCE_DIR`. Use it only as a comparison
  tie-breaker when functional evidence and readability are otherwise equivalent.
- [ ] Optionally record a pinned-tool cognitive-complexity result as audit metadata. Do
  not compare values from different tool versions. A high value triggers manual review,
  not an automatic deduction.
- [ ] Follow [`source-metrics.md`](source-metrics.md) for metric scope, interpretation,
  and reproducibility requirements.

## 5. Scope and fairness check

- [ ] Do not deduct for a different valid algorithm, private layout, file arrangement,
  or a concise design that differs from the reference.
- [ ] Do deduct for unrequested complexity that harms clarity: threads, plugin systems,
  ECS, broad callback/event frameworks, serialization, rendering, or mandatory external
  dependencies.
- [ ] Do not award credit for features outside the supplied task.
- [ ] Check that score reflects demonstrated behavior and source evidence, not prose,
  class count, or familiarity with a named algorithm.

## 6. Standard report

Use this structure for each candidate:

```text
Build/API gate: PASS | FAIL — evidence
Safety gate:    PASS | FAIL — evidence
Conformance tracks: G pass/total, N pass/total, S pass/total, C pass/total
Architecture and dependency design: __/20
Decomposition and complexity: __/20
C++ clarity and discipline: __/10
Tests and functional discipline: __/15
Functional conformance beyond the gate: __/35
Total: __/100 (cap applied? yes/no)

Strengths:
- ...

Material defects / unmet requirements:
- [Requirement ID] ...

Architecture and plan observations:
- ...

Commands and toolchain:
- ...

Supplemental source review:
- NCLOC: __ (command/scope)
- Duplication/unification findings: ...
- Optional pinned complexity result: __ (tool/version/config), or not collected
```
