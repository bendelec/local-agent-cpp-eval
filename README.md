# VWmini

VWmini is a compact C++23 implementation task for an immutable 2D triangle navmesh,
deterministic pathfinding, and local disc-agent avoidance.

## Repository layout

- [`candidate/`](candidate/) — the complete task package copied into an implementation
  workspace. It contains only the fixed public headers, requirements, CMake starter,
  and implementation prompt inputs.
- [`prompt/`](prompt/) — the implementation prompt injected by the harness.
- [`evaluator/`](evaluator/) — private reference implementation, conformance harness,
  review guidance, and optional SDL3 visual lab. Do not distribute this directory with
  `candidate/`.
- [`scripts/package-candidate.sh`](scripts/package-candidate.sh) — creates a clean
  task-package copy containing only `candidate/`.

## Private evaluator quick start

```sh
cmake -S evaluator -B build/reference
cmake --build build/reference --parallel
ctest --test-dir build/reference --output-on-failure
```

See [`evaluator/README.md`](evaluator/README.md) for the conformance runner and optional
visual lab.
