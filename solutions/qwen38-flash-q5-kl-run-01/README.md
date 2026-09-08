# VWmini

A compact, single-threaded C++23 library for an **immutable 2D navigation mesh**,
**deterministic point-to-point paths**, and **local avoidance between disc agents**. It has
no third-party dependencies, no global state, no threads, and no console output; every
invalid public call returns a diagnostic `Result`/`Error` instead of crashing.

## What is implemented

| Capability | Public entry point | Requirements |
|---|---|---|
| Value-semantic 2D vectors | `vwmini::Vec2`, `length`, `normalized` | MSH-001 |
| Ear-clip triangulation of one simple CCW outline | `vwmini::triangulate_simple_polygon` | MSH-002, MSH-003 |
| Immutable, validated triangle mesh with derived adjacency | `vwmini::NavMesh::create`, `cell_count` | MSH-004, MSH-005, MSH-007, MSH-008 |
| One containment policy (`epsilon = 1e-4f` boundary band) | `vwmini::NavMesh::contains` | MSH-006 |
| Deterministic, corridor-shortened routes | `vwmini::find_path` | SIM-001…SIM-004 |
| Agent lifecycle, goals, substepped motion | `vwmini::Simulation` | SIM-005…SIM-009, SIM-013 |
| Snapshot-based local avoidance | `vwmini::Simulation::step` | SIM-010…SIM-012 |

Out of scope by design (see `docs/requirements/requirements.md`): mesh edits, holes,
multiple maps, weighted routing, runtime obstacles, persistence, concurrency, callbacks,
rendering, and global multi-agent planning.

## Contract and design documents

* Requirements: `docs/requirements/requirements.md` (index), `mesh.md`, `simulation.md`,
  `non-functional.md`.
* Fixed public API: `docs/architecture/public-api.md`; the declarations themselves live in
  `include/vwmini/{geometry,nav_mesh,simulation}.hpp` and are unchanged.
* Design of this implementation: `docs/architecture/architecture.md`.
* Work breakdown and how the design changed during implementation:
  `docs/architecture/implementation-plan.md`.

## Layout

```
include/vwmini/   fixed public headers (the only public surface)
src/              implementation: predicates, triangulation, mesh_topology, corridor,
                  pathfinding, agent, steering, simulation, error
tests/            deterministic GTest suites + shared mesh fixtures
tests/consumer/   scratch consumer that links vwmini::vwmini like a real user
scripts/          check.sh (gate), fmt.sh (formatting)
```

## Build

```sh
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Consumers link the namespaced target `vwmini::vwmini`, which carries the C++23 requirement
and the `include/` path. The project is consumed through `add_subdirectory` (there are no
install rules; see `tests/consumer/CMakeLists.txt`):

```cmake
add_subdirectory("${VWMINI_SOURCE_DIR}" vwmini_build)
target_link_libraries(my_app PRIVATE vwmini::vwmini)
```

Tests are built when `VWMINI_BUILD_TESTS=ON` (the default in this repository) and are wired
into CTest.

## Verification gate

`scripts/check.sh [build-dir]` runs the whole gate in one go: a clean configure, a build
that must be warning-free under `-Wall -Wextra -Wpedantic` (plus the stricter warnings listed
in `CMakeLists.txt`), the full CTest suite, and `clang-format --dry-run -Werror` over
`include/`, `src/` and `tests/`. `scripts/fmt.sh` applies the formatting from
`.clang-format`.

The consumer smoke check is separate on purpose: it configures the consumer as its own
project, so it exercises the target and include path exactly as a downstream user would:

```sh
cmake -S tests/consumer -B build/consumer -DVWMINI_SOURCE_DIR=$PWD -DVWMINI_BUILD_TESTS=OFF
cmake --build build/consumer --parallel
./build/consumer/consumer && echo OK
```

Exit status 0 means the library behaves as documented when used through `vwmini::vwmini`.
