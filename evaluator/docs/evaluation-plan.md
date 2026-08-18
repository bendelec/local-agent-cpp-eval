# Evaluation Plan: Independent Tracks

This document is private. VWmini deliberately measures both geometric reasoning and
software engineering. Do not let a failure in one obscure evidence from the other.

## Test tracks

| Track | Purpose | Setup rule | Score area |
|---|---|---|---:|
| G — Geometry | Measure vector predicates, outline validation, and deterministic triangulation. | Calls only `geometry.hpp`; no `NavMesh`, `Simulation`, or crowd fixture. | 20 |
| N — Navmesh and paths | Measure triangle-mesh ingestion, topology, containment, and routes. | Uses fixed, hand-authored valid triangles; never calls `triangulate_simple_polygon`. | 20 |
| S — Simulation lifecycle | Measure agent ids, errors, routes, goals, stepping, and arrival. | Uses a known-good fixed mesh created directly from triangles. | 20 |
| C — Crowd behavior | Measure snapshot local avoidance and physical invariants. | Uses the fixed triangles in `reference-scenarios.md`; never calls geometry helpers. | 15 |
| T — Candidate tests | Measure test discipline and coverage. | Source/test review plus candidate tests. | 10 |
| A — Architecture | Measure modularity, ownership, dependency direction, and scope judgment. | Source review only; no prescribed file layout or algorithm. | 10 |
| Q — C++ quality | Measure idiomatic, warning-clean, maintainable C++23. | Build and source review. | 5 |

## Execution and reporting

Run each functional track independently and report a separate pass/fail result and
score. A failure in G must not prevent N, S, or C from compiling/running where their
public API dependencies are otherwise present. A failure in C must not erase earned
points from G, N, or S.

The overall 100-point score is the sum of tracks, subject only to the global safety
cap in `evaluation-rubric.md`. Report the vector of track scores as well as the total;
for example, `G 16/20, N 19/20, S 18/20, C 10/15, T 7/10, A 9/10, Q 5/5 = 84/100`.

## Fixture discipline

- Use ordinary, well-separated finite geometry in public and private fixtures.
- Keep test meshes as literal triangles in N, S, and C; do not create them with the
  candidate triangulator.
- Do not assume a particular triangulation order beyond the stated determinism
  contract, path tie winner, avoidance passing side, private class layout, or source
  file structure.
- A candidate must not gain credit for recognizing fixture coordinates. Randomized
  translations/rotations of equivalent fixtures may be used as anti-hardcoding checks.
