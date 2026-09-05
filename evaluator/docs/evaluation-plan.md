# Evaluation Plan: Independent Tracks

This is published evaluator guidance; it is not supplied in a candidate task package.
VWmini deliberately measures both geometric reasoning and software engineering. Do not
let a failure in one obscure evidence from the other.

## Test tracks

| Track | Purpose | Setup rule |
|---|---|---|
| G — Geometry | Measure vector predicates, outline validation, and deterministic triangulation. | Calls only `geometry.hpp`; no `NavMesh`, `Simulation`, or crowd fixture. |
| N — Navmesh and paths | Measure triangle-mesh ingestion, topology, containment, and routes. | Uses fixed, hand-authored valid triangles; never calls `triangulate_simple_polygon`. |
| S — Simulation lifecycle | Measure agent ids, errors, routes, goals, stepping, and arrival. | Uses a known-good fixed mesh created directly from triangles. |
| C — Crowd behavior | Measure snapshot local avoidance and physical invariants. | Uses the fixed triangles in `reference-scenarios.md`; never calls geometry helpers. |

## Execution and reporting

Run each functional track independently and report a separate pass/fail result. A failure
in G must not prevent N, S, or C from compiling/running where their public API
dependencies are otherwise present. A failure in C must not erase evidence from G, N, or
S.

The four tracks are evidence for the **35-point Functional conformance beyond the gate**
category; they are not separately weighted score buckets. The canonical 100-point model
is `evaluation-rubric.md`: architecture/dependency design 20, decomposition/complexity
20, C++ clarity/discipline 10, tests/functional discipline 15, and functional
conformance 35, subject to the safety cap. Report the rubric-category vector and the
four track outcomes in every evaluation record.

## Fixture discipline

- Use ordinary, well-separated finite geometry for baseline behavior. Add finite extreme-scale
  cases only when they directly exercise an explicit numerical contract, and document the
  contract and oracle.
- Keep test meshes as literal triangles in N, S, and C; do not create them with the
  candidate triangulator.
- Do not assume a particular triangulation order beyond the stated determinism
  contract, path tie winner, avoidance passing side, private class layout, or source
  file structure.
- A candidate must not gain credit for recognizing fixture coordinates. Randomized
  translations/rotations of equivalent fixtures may be used as anti-hardcoding checks.
