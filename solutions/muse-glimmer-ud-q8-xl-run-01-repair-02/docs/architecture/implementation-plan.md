# Implementation Plan

## Work Packages

### WP1 – Geometry primitives
Goal: Vec2 ops, length, normalized, epsilon constants.
Files: `include/vwmini/geometry.hpp` (public, no change), `src/vwmini.cpp`
Verification: unit tests for Vec2 arithmetic, length, normalized zero.
Status: done

### WP2 – Triangulation
Goal: `triangulate_simple_polygon` validation + ear clipping.
Dependencies: WP1
Files: `src/vwmini.cpp`
Verification: tests for valid CCW polygon → deterministic triangles, area sum within tolerance; invalid cases → InvalidArgument/InvalidMesh.
Status: done

### WP3 – NavMesh core
Goal: `NavMesh::create`, `contains`, `cell_count`
Dependencies: WP1, WP2
Files: `src/vwmini.cpp`
Verification: valid triangles accepted, invalid cases rejected, contains boundary epsilon.
Status: done

### WP4 – Pathfinding
Goal: `find_path` with validation, segment containment test, triangle adjacency, Dijkstra, string pulling.
Dependencies: WP3
Files: `src/vwmini.cpp`
Verification: direct segment, disconnected, containment of each segment, determinism.
Status: done

### WP5 – Simulation core
Goal: Agent storage, add/remove, set_goal/clear_goal, state queries.
Dependencies: WP4
Files: `src/vwmini.cpp`
Verification: config validation, arrival radius sentinel, status transitions.
Status: done

### WP6 – Stepping & avoidance
Goal: `step` validation, waypoint following, max speed, arrival, substepping, snapshot avoidance.
Dependencies: WP5
Files: `src/vwmini.cpp`
Verification: movement bounds, containment, no overshoot, collision avoidance in open cell.
Status: done

### WP7 – Build & tests
Goal: CMake target, deterministic tests, formatting.
Dependencies: all
Files: `CMakeLists.txt`, `tests/`
Verification: `cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure`
Status: done

## Slices

#### S1 – Vec2 & helpers
Goal: implement `length` and `normalized`.
Files: `src/vwmini.cpp`
Verification: tests pass.
Completion: done

#### S2 – Polygon validation
Goal: non-finite, vertex count, duplicate consecutive, winding, self-intersection.
Verification: ...
Completion: done

#### S3 – Ear clipping
Goal: deterministic ear removal, non-degenerate triangles.
Verification: ...
Completion: done

#### S4 – NavMesh validation
Goal: triangle validation, non-manifold, T-junction, overlap detection.
Verification: ...
Completion: done

#### S5 – Contains
Goal: point in triangle + edge distance epsilon.
Verification: ...
Completion: done

#### S6 – Segment containment
Goal: intersection sampling test.
Verification: ...
Completion: done

#### S7 – Triangle adjacency
Goal: edge map with epsilon matching.
Verification: ...
Completion: done

#### S8 – Triangle search & Dijkstra
Goal: find containing triangle, Dijkstra with deterministic tie-break.
Verification: ...
Completion: done

#### S9 – String pulling path
Goal: initial portal points + shortcut with segment containment.
Verification: ...
Completion: done

#### S10 – Simulation agent management
Goal: add/remove/set/clear, validation.
Verification: ...
Completion: done

#### S11 – Path following step
Goal: move along polyline, arrival, status.
Verification: ...
Completion: done

#### S12 – Avoidance snapshot
Goal: substepping, pairwise separation.
Verification: ...
Completion: done

## Revision Log
* Initial plan created.
* 2025-09-18: All work packages completed; build and tests pass.
