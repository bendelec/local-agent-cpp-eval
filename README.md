# VWmini — Long-Running Agentic C++ Benchmark

VWmini is a deliberately small-to-mid-sized C++23 implementation task used for a
**qualitative, subjective comparison of software-development agents**. It is not a
single-number leaderboard and it does not measure only whether a model makes a test suite
pass. It evaluates the submitted library: whether it is correct, robust, and maintainable
when assessed through conformance, tests, source review, and architecture review.

The task is an immutable 2D triangle navigation mesh with deterministic pathfinding and
local disc-agent avoidance. It is compact enough to inspect carefully, but broad enough to
require geometry, numerical judgment, API design, CMake, testing, documentation, and
stateful simulation work.

The primary comparison set is models that can be hosted locally on a machine with 128 GB
of unified memory—currently a Strix Halo system. A small number of explicitly labelled
comparison runs fall outside that constraint. For example, the hosted unquantized
DeepSeek V4 Flash run is retained to show how much the aggressive Antirez GGUF
quantization used by the locally served Dwarfstar DS4 version affects the outcome. See the
[model evaluation overview](evaluations/overview.md) for the recorded runs and results.

## Evaluation boundary

VWmini and [Reflective Pi](https://github.com/bendelec/reflective-pi) use the same
standard task runs, but answer different questions. The boundary is intentional:

| Repository | What it evaluates |
|---|---|
| **VWmini** | The delivered C++ library: requirements conformance, build and test evidence, numerical and state-machine behavior, architecture, code clarity, complexity, and documentation accuracy. |
| **Reflective Pi** | The run as an agentic process: session quality, context curation and compaction, subagent use, tool use, and related meta-cognitive behavior. |

A VWmini score is based only on the delivered task outcome and its engineering evidence.
It does **not** score context management, the presence or quality of subagents, or other
session-process behavior. A run may be long and span multiple context windows, but that
fact is workload context rather than a VWmini scoring dimension. Evaluation records may
preserve compact, explicitly operator-attributed run-accounting context when it helps
interpret a result; such context is never scored. Detailed qualitative session analysis
remains in Reflective Pi or a separate restricted record.

## Run-control policy

For every published comparison run, the operator requests a **high** thinking/reasoning
setting when the serving interface exposes that named level. When it does not, “nearest
semantic equivalent” means the available user-visible thinking/reasoning mode nearest to
high in the provider's own documented ordering. It is not a claim that providers implement
equal internal reasoning budgets or compute. The per-provider setting mapping and actual
serving-side setting are not retained as run artifacts, so this is a statement of the common
run-control policy, not independently verifiable per-run telemetry. In particular, a
token-volume or wall-time difference between Muse and Qwen must not be read as deliberately
running Muse at a low thinking level while running Qwen at an extra-high level. This last
comparison is an operator attestation about the run setup, not independently reproducible
per-run telemetry.

## What VWmini evaluates

- **Maintainable native C++ development.** Can the agent build a clear C++23 library with
  cohesive boundaries, understandable ownership, value semantics, deterministic behavior,
  proportionate function/control-flow complexity, CMake integration, and focused tests—
  not merely generate a plausible-looking implementation?
- **Geometric reasoning and problem solving.** Polygon validation, triangle topology,
  boundary tolerances, portal routing, and local avoidance expose weaknesses that are easy
  to miss in CRUD-style or purely web-based benchmark tasks.
- **Evidence-driven engineering.** Can the implementation withstand failures, visual
  diagnostics, sanitizer findings, and review feedback without accumulating special cases
  and patch layers?
- **Functional correctness beyond example tests.** Does the public API meet the specified
  geometry, pathfinding, lifecycle, and crowd contracts under independent tests and
  targeted regression probes?

The choice of a native C++ geometry task is intentional. Many public coding comparisons
focus on small tasks that fit in one context window, frequently in browser-hosted
environments. VWmini instead favors an inspectable repository workflow and continuous
geometry and simulation behavior where superficially passing examples can hide important
defects.

## How a run is evaluated

Evaluation is intentionally layered. A passing test count is necessary evidence, but it
is not conclusive proof of correctness and is not the benchmark's primary measure. The
central question is whether the model produced a correct *and maintainable*
implementation: coherent architecture, sensible decomposition, low accidental coupling,
clear C++ ownership and control flow, and tests that genuinely establish the claimed
behavior. Functional conformance remains a substantial prerequisite and source of
evidence rather than a substitute for those qualities.

1. **Immutable source snapshot.** When a model finishes, its workspace is copied into a
   `solutions/` archive. Generated build output is excluded and the source snapshot is
   never rewritten.
2. **Black-box conformance.** The provider is built through its public
   `vwmini::vwmini` target and exercised by independently labelled geometry, navmesh/path,
   simulation, and crowd tracks.
3. **Model-authored tests and build quality.** We run the submitted native CTest suite,
   warning-clean compiler builds, and, where appropriate, sanitizer builds.
4. **Source, architecture, and complexity review.** Review considers cohesion and
   dependency boundaries, shared-policy consistency, function/control-flow complexity,
   ownership, API contracts, numerical and state-machine reasoning, scope control,
   documentation accuracy, and whether tests genuinely prove their claims.
5. **Visual and exploratory diagnostics.** The optional SDL3 lab links only against the
   public API. It can reveal integration failures—such as a route that is valid on paper
   but stalls when followed—that automated fixtures did not yet cover.
6. **Revision of the evidence.** When a visual diagnosis or review finds a general,
   reproducible defect, a public-API regression is added to the evaluator suite. Earlier
   reports explain any resulting score revision rather than leaving optimistic results in
   place.

Scores are **subjective composite review scores** against the published rubric, not a
claim of absolute or universal model capability. They deliberately favor a smaller,
direct, well-reasoned implementation over a larger test-passing patchwork. The overview
records track outcomes and the rationale for each composite score.

## Published evaluator and candidate isolation

This repository publishes the current evaluator, reference implementation, fixtures, and
rubric so that reported results can be reproduced. They are **not** copied into a model's
task workspace: [`candidate/`](candidate/) and the implementation prompt are the only
task inputs supplied by the harness.

Publication makes this evaluator revision a known suite. It remains black-box in the
sense that it consumes only the candidate's public API, but it is not a held-out suite for
future runs. A controlled blind evaluation must use an unreleased evaluator revision and
publish it only after the evaluated runs are complete.

## Repository layout

- [`candidate/`](candidate/) — the task package copied into a model implementation
  workspace: fixed public headers, requirements, and CMake starter material.
- [`prompt/`](prompt/) — the implementation prompt injected by the harness.
- [`evaluator/`](evaluator/) — the published reference implementation, black-box
  conformance harness, review guidance, benchmark fixtures, and optional SDL3 visual
  lab. It is evaluator material, not part of a candidate submission.
- [`solutions/`](solutions/) — immutable model-produced source snapshots, including
  explicitly named repair lineage where applicable.
- [`evaluations/`](evaluations/) — reviewer reports, score rationale, repair briefs, and
  the [comparison overview](evaluations/overview.md). These are kept separate from
  model-produced snapshots.
- [`sessions/`](sessions/) — sanitized run provenance only. Raw session analysis belongs
  in the Reflective Pi project or an external restricted archive.
- [`scripts/package-candidate.sh`](scripts/package-candidate.sh) — creates a clean task
  package containing only `candidate/`.

## Recording model runs

Use one stable lowercase base run id, for example `model-local-q4-run-01`. A run may
contain an initial model output and zero or more separately archived repair attempts:

```text
solutions/<run-id>/              initial immutable model output, if retained
solutions/<run-id>-repair-01/    immutable first repair output, if any
evaluations/<run-id>.md          report that identifies the exact scored source tree
sessions/<run-id>/               sanitized run provenance
```

The evaluation record—not a directory-name convention—names the authoritative scored
source tree and its fingerprint. Do not rewrite model-produced source during evaluation.
Put reviewer notes, reproduced commands, diagnostics, and grading evidence in the
separate evaluation record.

A scored source snapshot retains its project root and `vwmini::vwmini` target, allowing
it to be checked directly:

```sh
./evaluator/conformance/run.sh solutions/<scored-run-tree> /tmp/vwmini-conformance
```

## Local evaluator quick start

Prerequisites are CMake 3.23 or newer, a C++23 compiler, and a CMake-discoverable
GoogleTest package (`find_package(GTest CONFIG REQUIRED)`). SDL3 is required only for
the optional visual lab.

```sh
cmake -S evaluator -B build/reference -DCMAKE_BUILD_TYPE=Debug
cmake --build build/reference --parallel
ctest --test-dir build/reference --output-on-failure
```

See [`evaluator/README.md`](evaluator/README.md) for the conformance runner, reference
build options, and optional visual lab.
