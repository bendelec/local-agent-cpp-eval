# VWmini

VWmini is a compact C++23 implementation task for an immutable 2D triangle navmesh,
deterministic pathfinding, and local disc-agent avoidance. Its purpose is to compare
LLMs as software developers: not only whether they produce a functioning library, but
how well they plan work, choose architecture and seams, manage ownership, write tests,
and keep a focused implementation understandable.

## Repository layout

- [`candidate/`](candidate/) — the complete task package copied into an implementation
  workspace. It contains only the fixed public headers, requirements, and CMake starter;
  the separately stored implementation prompt is injected by the harness.
- [`prompt/`](prompt/) — the implementation prompt injected by the harness.
- [`evaluator/`](evaluator/) — private reference implementation, conformance harness,
  review guidance, and optional SDL3 visual lab. Do not distribute this directory with
  `candidate/`.
- [`solutions/`](solutions/) — exact final workspaces produced by the tested models;
  each remains directly linkable/testable as a CMake provider.
- [`evaluations/`](evaluations/) — reviewer reports and scores, kept separate from the
  model-produced implementation snapshots.
- [`sessions/`](sessions/) — retained Pi session exports and run metadata for provenance,
  wall-clock/context/tool analysis, and reproducibility.
- [`scripts/package-candidate.sh`](scripts/package-candidate.sh) — creates a clean
  task-package copy containing only `candidate/`.

## Recording model runs

Use one stable lowercase run id everywhere, for example `ds4f-local-q4-run-01`.

```text
solutions/<run-id>/       exact completed project copied from the model workspace
evaluations/<run-id>.md   conformance results, review evidence, and score
sessions/<run-id>/        Pi session export plus model/hardware/run metadata
```

A solution snapshot must retain its project root and `vwmini::vwmini` target unchanged,
so it can be checked directly:

```sh
./evaluator/conformance/run.sh solutions/<run-id> /tmp/vwmini-<run-id>
```

Do not rewrite a model-produced solution while evaluating it. Place reviewer notes,
reproduced commands, and grading evidence in its separate evaluation record.

## Private evaluator quick start

```sh
cmake -S evaluator -B build/reference
cmake --build build/reference --parallel
ctest --test-dir build/reference --output-on-failure
```

See [`evaluator/README.md`](evaluator/README.md) for the conformance runner and optional
visual lab.
