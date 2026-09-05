# VWmini — Implementation Plan

Work is divided into ordered packages, each producing a small verifiable slice. Every
slice lists goal, dependencies, expected files/interfaces, verification, and status.

## WP1 — Core geometry helpers + scaffolding
- Goal: provide shared geometry primitives and build the CMake skeleton.
- Dependencies: public headers (unmodified).
- Files: `src/vwmini/detail.hpp`; `src/vwmini/geometry.cpp`; update `CMakeLists.txt`
  to compile private sources and register GTest.
- Interfaces: `detail::epsilon`, `detail::signed_area`,
  `detail::point_in_triangle_strict`, `detail::point_in_triangle`,
  `detail::dist_point_segment`, `detail::segments_intersect`,
  `detail::finite(Vec2)`.
- Verification: compiles warning-clean (`-Wall -Wextra -Wpedantic`); a `length`/
  `normalized` unit test passes.
- Status: pending.

## WP2 — Triangulation
- Goal: implement `triangulate_simple_polygon` (ear-clipping) with full validation.
- Dependencies: WP1.
- Files: `src/vwmini/triangulate.cpp`.
- Verification: tests for non-finite→`InvalidArgument`, <3 / duplicate /
  self-intersect / CW / degenerate → `InvalidMesh`, valid CCW polygon → deterministic
  non-overlapping CCW triangles with matching signed area.
- Status: pending.

## WP3 — NavMesh construction, containment, adjacency
- Goal: `NavMesh::Impl`, `create`, `contains`, `cell_count`.
- Dependencies: WP1.
- Files: `src/vwmini/navmesh_detail.hpp`, `src/vwmini/nav_mesh.cpp`.
- Verification: tests for each `MSH-004` rejection (non-finite, empty, non-triangle, CW,
  degenerate, overlap, non-manifold edge, T-junction), `contains` boundary policy,
  `cell_count`, transactional failure (no partial mesh).
- Status: pending.

## WP4 — find_path (funnel)
- Goal: cell location → Dijkstra corridor → apex funnel → taut contained polyline.
- Dependencies: WP3.
- Files: `src/vwmini/path.cpp`.
- Verification: exact equal endpoints→`[start]`; same-cell→`[start,goal]`;
  non-finite→`InvalidArgument`; out-of-mesh→`OutsideMesh`; disconnected→`NoPath`;
  every output segment contained by mesh (sampled) and shorter than cell-centre path;
  determinism across repeats.
- Status: pending.

## WP5 — Simulation: lifecycle + goals
- Goal: add/remove agents, `set_goal`/`clear_goal` state transitions.
- Dependencies: WP3.
- Files: `src/vwmini/simulation.cpp`.
- Verification: id validation, config validation, `OutsideMesh` on bad position,
  arrival-radius sentinel (`-1` uses radius; other negatives→`InvalidArgument`),
  goal state transitions (Moving/NoPath/Reached/Idle), `NotFound` for removed ids.
- Status: pending.

## WP6 — Simulation: stepping + local avoidance
- Goal: `step` sub-stepped motion, waypoint overshoot guard, arrival, snapshot-based
  reciprocal avoidance keeping agents in mesh and within max speed.
- Dependencies: WP5 + WP4.
- Files: `src/vwmini/simulation.cpp` (avoidance).
- Verification: open-cell two-agent non-overlap `<= 1e-3`, progress toward goals,
  max-speed bound, mesh containment, transactional invalid-step, zero-step no-op.
- Status: pending.

## WP7 — Polish & full verification
- Goal: clang-format clean, clang-tidy warnings addressed, full test run.
- Files: formatting of all sources/tests; CMake tidy target.
- Verification: `cmake --build`, `ctest` all pass, `clang-format --dry-run` clean,
  `clang-tidy` clean.
- Status: pending.
