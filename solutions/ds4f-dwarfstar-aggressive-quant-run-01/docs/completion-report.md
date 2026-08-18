# VWmini — Completion Report

## Summary

Implemented **VWmini** from scratch: a compact native C++23 library providing an immutable 2D navmesh, deterministic point-to-point paths, and local disc-agent avoidance. The fixed public API in `include/vwmini/` and the `vwmini::vwmini` CMake target were preserved unchanged; only private implementation details were added to headers.

## Design / Modules

Three single-responsibility modules with dependency flow inward (`geometry → navmesh → simulation`), each owning its private `Impl` (referenced by pointer/shared_ptr from the public headers to preserve the fixed API).

| Module | Files | Responsibility |
|---|---|---|
| `geometry` | `src/geometry.cpp` | `Vec2`/`Polygon` ops, `triangulate_simple_polygon` via deterministic ear-clipping. |
| `navmesh` | `src/internal.hpp`, `src/nav_mesh.cpp` | Shared helpers; `NavMesh::create` validation, `contains`, `cell_count`, `find_path` (cell A* corridor + visibility-graph Dijkstra corridor shortening). |
| `simulation` | `src/simulation.cpp` | Move-only `Simulation`: agent lifecycle, substep motion, snapshot-based reciprocal swept-segment avoidance. |

### Key architectural seams (documented in `docs/architecture/architecture.md`)
- **Validation seam**: every public input is validated; `Result<T>`/`Error` with the fixed `ErrorCode` values; never crashes or silently repairs invalid geometry.
- **Boundary-policy seam**: `contains` uses strict interior + closed-edge distance ≤ epsilon.
- **Adjacency seam**: adjacency only via complete non-zero shared edges with endpoints ≤ epsilon apart; vertex-only touch creates no adjacency.
- **Determinism seam**: all algorithms are deterministic; identical input → identical output (triangulation, `cell_count`, paths, simulation runs).

### Avoidance design (final)
Reciprocal **swept-segment** prediction: each agent evaluates candidate velocity offsets against the *other agent's swept segment* (its snapshot position → predicted position over the lookahead horizon at snapshot velocity, `kHorizon = 0.5f`), so agents swerve early. `kOffsets` are ordered to prefer large lateral swerves (90°/-90°/45°/-45°/…) once forward motion is blocked, so head-on/crossing agents diverge symmetrically without deadlock. Stop (`{0,0}`) is the fallback, guaranteeing no overlap (SIM-012 permits stopping when boxed).

## Files Changed

- `src/vwmini.cpp`, `src/geometry.cpp`, `src/internal.hpp`, `src/nav_mesh.cpp`, `src/simulation.cpp` — implementations.
- `CMakeLists.txt`, `tests/CMakeLists.txt` — library sources and three test executables.
- `tests/geometry_test.cpp`, `tests/nav_mesh_test.cpp`, `tests/simulation_test.cpp` — deterministic tests.
- `docs/architecture/architecture.md` — architecture doc (required, created from scratch, kept current).
- `.clang-format` — formatting config (LLVM-based, 120 column limit).

## Build & Test Commands + Outcomes

```
cmake -S . -B build && cmake --build build --parallel
```
Clean build, **no compiler warnings** (`-Wall -Wextra -Wpedantic`).

```
ctest --test-dir build
```
```
1/3 geometry_test .... Passed
2/3 navmesh_test ..... Passed
3/3 sim_test ......... Passed
100% tests passed out of 3
```
Individual executables: `./build/tests/vwmini_geometry_test`, `./build/tests/vwmini_navmesh_test`, `./build/tests/vwmini_sim_test` — all pass.

**Formatting**: all `src/*.cpp`, `src/*.hpp`, `include/vwmini/*.hpp`, `tests/*.cpp` pass `clang-format --dry-run --Werror` (clean).

## Deliberately-Chosen Behaviors (contract permitted choices)

1. **Ear-clipping triangulation** over alternatives — simple, deterministic, matches the single-simple-polygon authoring helper; strict ear test avoids T-junctions so output feeds `NavMesh::create` cleanly.
2. **Visibility-graph Dijkstra corridor shortening instead of the full funnel** — builds a visibility graph over corridor vertices + start/goal and runs Dijkstra to shorten the A* corridor; satisfies SIM-003 and is provably contained, with lower subtle-bug risk than the textbook funnel.
3. **Dense sampling for the direct-segment check** (SIM-001) — deterministic and effectively exact for non-ambiguous geometry.
4. **Same-cell endpoints → `[start, goal]` directly** — avoids inconsistency between the sampling check and the corridor path in the tolerance-ambiguous single-cell case.
5. **Substep loop with snapshot avoidance** (SIM-008/010/012) — all agents snapshot, decide, then apply displacements simultaneously.
6. **Static-disc → reciprocal swept-segment avoidance** — velocity-prediction caused deadlock/collision at close separation; swept-segment prediction with a lookahead horizon makes head-on/crossing agents diverge deterministically. Stop-fallback guarantees no overlap.
7. **Arrival-radius rule** (SIM-006): only `-1.0f` is the negative sentinel; effective arrival = radius when `-1.0f`, else the given radius; within effective arrival → `Reached`.
8. **`routeIndex` starts at 1** in `assignGoal` (skips `route[0]`, the already-occupied start point) — fixes multi-substep oscillation toward the start.
9. **`nextId` wraps to 1 after `UINT32_MAX`** to avoid overflow UB.
10. **Numerical policy**: epsilon `1e-4f` applies only to point/edge distance and shared-edge endpoint matching; triangle non-degenerate iff absolute signed double area > `epsilon²`; ambiguous inputs separated ≤ epsilon may accept or reject but stay safe and deterministic.
