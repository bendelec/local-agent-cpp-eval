# VWmini implementation plan

Status values: **planned**, **in progress**, **complete**, or **revised**.  All slices below
are complete. The completed implementation follows the component and visibility-routing decision
documented before source edits; WP5 records the focused numeric/avoidance repair.

## WP5 — Numeric stepping and feasible local avoidance repair

This package is a **revision** prompted by post-completion valid-input review findings.
It preserves the existing pImpl/dependency design while separating intended velocity,
mesh-feasible displacement, representable stored endpoint, and committed velocity.

1. **R1: finite, representable motion** — **complete**  
   Goal: make every positive finite substep terminate and commit only a finite stored displacement
   within its elapsed-speed budget. Dependencies: S2. Files/interfaces: `src/simulation.cpp`,
   `tests/vwmini_tests.cpp`. Verification: denormal and far-origin one-ULP regressions.
2. **R2: feasible snapshot avoidance** — **complete**  
   Goal: evaluate rounded mesh-feasible endpoint candidates from one snapshot before deterministic
   pair separation selection, retaining progress choices. Dependencies: R1. Files/interfaces:
   `src/simulation.cpp`, `tests/vwmini_tests.cpp`. Verification: crossing and corner-following
   completion/separation regressions.
3. **R3: formatting and multi-toolchain evidence** — **complete**  
   Goal: add checked formatting configuration, apply it, and run the requested clean builds,
   sanitizers where available, library-only build, and formatting check. Dependencies: R1–R2.
   Files/interfaces: `.clang-format`, all C++ files, CMake/tests/docs. Verification: recorded
   command outcomes in the completion report.

## WP1 — Foundation and geometry

1. **G1: project/test scaffold** — **complete**  
   Goal: configure the C++23 target and a deterministic CTest executable.  
   Dependencies: none. Files/interfaces: `CMakeLists.txt`, `tests/`.  
   Verification: configure, build, and run a smoke test.
2. **G2: vector and polygon triangulation** — **complete**  
   Goal: implement finite validation, simple-outline checks, and deterministic CCW ear
   clipping.  Dependencies: G1. Files/interfaces: `src/geometry.cpp`, `geometry.hpp`.
   Verification: duplicate, winding, intersection, collinear, and area tests.

## WP2 — Immutable mesh and paths

1. **M1: transactional mesh validation/storage** — **complete**  
   Goal: validate triangles, intersection/topology/T-junction rules, and immutable
   adjacency/components. Dependencies: G2. Files/interfaces: `src/nav_mesh.cpp`, private
   `NavMesh::Impl`. Verification: invalid mesh and containment tests.
2. **M2: deterministic contained routing** — **complete**  
   Goal: implement direct-segment detection and component-constrained, string-pulled
   visibility routing. Dependencies: M1. Files/interfaces: `src/nav_mesh.cpp`.
   Verification: direct, bend, disconnected, and vertex-touch path tests.

## WP3 — Simulation

1. **S1: agent lifecycle and goals** — **complete**  
   Goal: implement private agent ownership, all validation, state transitions, and query
   snapshots. Dependencies: M2. Files/interfaces: `src/simulation.cpp`, private
   `Simulation::Impl`. Verification: lifecycle/error/NoPath tests.
2. **S2: bounded simultaneous motion and avoidance** — **complete**  
   Goal: fixed substeps, route following, mesh checks, and deterministic local disc
   avoidance. Dependencies: S1. Files/interfaces: `src/simulation.cpp`. Verification:
   large-step, arrival, crossing, overtaking, and invalid-step transactional tests.

## WP4 — Integration quality

1. **Q1: quality/documentation pass** — **complete**  
   Goal: warning-clean build, full deterministic tests, and documentation/code consistency.
   Dependencies: G1–S2. Files/interfaces: all changed files and both architecture docs.
   Verification: `cmake -S . -B build`, build with warnings, and `ctest`.
