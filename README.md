# VWmini — Long-Running Agentic C++ Benchmark

VWmini is a deliberately small-to-mid-sized C++23 implementation task used for a
**qualitative, subjective comparison of software-development agents**. It is not a
single-number leaderboard and it is not intended to measure only whether a model can
make a test suite pass. The goal is to observe how an agent works through a realistic,
long-running implementation task: planning, implementation, verification, review, and
course correction.

The task is an immutable 2D triangle navigation mesh with deterministic pathfinding and
local disc-agent avoidance. It was selected because it is compact enough to inspect
carefully, but broad enough to require geometry, numerical judgment, API design, CMake,
testing, documentation, and stateful simulation work.

## What this benchmark is trying to evaluate

A normal VWmini run is intentionally expected to span **at least three to five full
model context windows**. It may consume more when an agent makes poor use of its context
or delegates inefficiently. That makes the run useful for observing capabilities that
short, one-context coding exercises often do not reveal:

- **Long-horizon planning.** Can the agent decompose a multi-stage task, establish an
  architecture before implementation, make dependencies explicit, and continue a plan
  after compaction without losing essential constraints?
- **Context management.** Can it preserve the right decisions, evidence, and open work
  across a long session rather than repeatedly rediscovering the task or carrying
  irrelevant detail forward?
- **Subagent use.** Can it delegate focused investigation, review, or verification work
  efficiently, then integrate the results rather than treating delegation as ceremony?
- **Native C++ development.** Can it build a maintainable C++23 library with clear
  ownership, value semantics, deterministic behavior, CMake integration, and focused
  tests—not merely generate a plausible-looking implementation?
- **Geometric reasoning and problem solving.** Polygon validation, triangle topology,
  boundary tolerances, portal routing, and local avoidance expose weaknesses that are
  easy to miss in CRUD-style or purely web-based benchmark tasks.
- **Evidence-driven engineering.** Can it respond to failures, visual diagnostics,
  sanitizer findings, and review feedback with minimal, well-tested corrections?

The choice of a native C++ geometry task is intentional. Many public coding comparisons
focus on small tasks that fit in one context window, frequently in browser-hosted
environments. VWmini instead favors an inspectable repository workflow and includes
continuous geometry and simulation behavior where superficially passing examples can
hide important defects.

## Relationship to Reflective Pi

The same runs also serve as a proof-of-concept workload for
[Reflective Pi](https://github.com/bendelec/reflective-pi), particularly its support for
active context curation during long agentic work. VWmini records the software-development
outcome and its evidence. The separate analysis of context-curation behavior, session
quality, and Reflective Pi itself belongs in the Reflective Pi repository rather than
here.

In other words, one run can answer two related but distinct questions:

1. How well did the model develop the VWmini library?
2. How well did the agent/runtime preserve and curate useful context throughout the run?

Only the first is documented and scored in this repository.

## How a run is evaluated

Evaluation is intentionally layered. A passing test count is necessary evidence, but it
is not treated as conclusive proof of correctness.

1. **Immutable source snapshot.** When a model finishes, its workspace is copied into
   `solutions/<run-id>/`. Generated build output is excluded and the snapshot is never
   rewritten.
2. **Black-box conformance.** The provider is built through its public
   `vwmini::vwmini` target and exercised by independently labelled geometry, navmesh/path,
   simulation, and crowd tracks.
3. **Model-authored tests and build quality.** We run the submitted native CTest suite,
   warning-clean compiler builds, and, where appropriate, sanitizer builds.
4. **Source and architecture review.** Review considers correctness beyond fixtures,
   numerical and state-machine reasoning, ownership, API contracts, scope control,
   documentation accuracy, and whether tests genuinely prove their claims.
5. **Visual and exploratory diagnostics.** The optional SDL3 lab links only against the
   public API. It is useful for finding integration failures—such as a route that is
   valid on paper but stalls when followed—that automated fixtures did not yet cover.
6. **Revision of the evidence.** When a visual diagnosis or review finds a general,
   reproducible defect, a public-API regression is added to the evaluator suite. Earlier
   results and scores are then updated rather than left artificially optimistic.

Scores therefore express reviewer judgment against the published rubric, not a claim of
absolute or universal model capability. Test suites and reviews can improve over time;
the evaluation records retain the evidence and explain score changes.

## Repository layout

- [`candidate/`](candidate/) — the task package copied into a model implementation
  workspace: fixed public headers, requirements, and CMake starter material. The
  separately stored implementation prompt is injected by the harness.
- [`prompt/`](prompt/) — the implementation prompt used by the harness.
- [`evaluator/`](evaluator/) — reference implementation, black-box conformance harness,
  review guidance, benchmark fixtures, and the optional SDL3 visual lab. It is evaluator
  material, not part of a candidate submission.
- [`solutions/`](solutions/) — immutable final workspaces produced by tested models;
  each remains directly buildable as a CMake provider.
- [`evaluations/`](evaluations/) — reviewer reports, score rationale, repair briefs,
  and the comparison overview. These are kept separate from model-produced snapshots.
- [`sessions/`](sessions/) — run metadata and any retained session provenance. Detailed
  context-curation analysis is intentionally kept in the Reflective Pi project.
- [`scripts/package-candidate.sh`](scripts/package-candidate.sh) — creates a clean task
  package containing only `candidate/`.

## Recording model runs

Use one stable lowercase run id everywhere, for example `model-local-q4-run-01`.

```text
solutions/<run-id>/       exact completed project copied from the model workspace
evaluations/<run-id>.md   conformance results, review evidence, and score
sessions/<run-id>/        run metadata and retained session provenance
```

A solution snapshot retains its project root and `vwmini::vwmini` target, allowing it to
be checked directly:

```sh
./evaluator/conformance/run.sh solutions/<run-id> /tmp/vwmini-<run-id>
```

Do not rewrite a model-produced solution during evaluation. Put reviewer notes,
reproduced commands, diagnostics, and grading evidence in its separate evaluation
record.

## Local evaluator quick start

```sh
cmake -S evaluator -B build/reference
cmake --build build/reference --parallel
ctest --test-dir build/reference --output-on-failure
```

See [`evaluator/README.md`](evaluator/README.md) for the conformance runner and optional
visual lab.
