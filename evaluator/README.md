# Evaluator Material

This repository publishes this evaluator revision so reported results can be reproduced.
The harness does **not** distribute it to a candidate: it copies only `candidate/` into
each candidate workspace and injects `prompt/standard-implementation-prompt.md` as text.
Consequently, this published revision is a known regression suite, not a held-out suite
for future blind runs. Keep any unreleased acceptance material outside a public checkout
until the associated evaluations are complete.

## Components

- `reference/` — clean C++23 reference implementation of the canonical public headers,
  plus 67 fast GTest unit/integration tests.
- `conformance/` — black-box, public-API-only headless tests. Its four independently
  labelled tracks are geometry, navmesh/path, simulation lifecycle, and crowd behavior.
- `lab/` — optional SDL3 visual diagnostic/editor, linked only through the public API.
- `docs/evaluation-plan.md` — independent track policy.
- `docs/evaluation-rubric.md` — 100-point source-review rubric.
- `docs/evaluation-checklist.md` — repeatable intake, test, and source-review procedure.
- `docs/benchmark/reference-scenarios.md` — crowd-fixture rationale and scenarios.

## Prerequisites

The evaluator requires CMake 3.23 or newer, a C++23 compiler, and a
CMake-discoverable GoogleTest installation (`find_package(GTest CONFIG REQUIRED)`). SDL3
is needed only when building the visual lab.

## Reference build and tests

```sh
cmake -S evaluator -B build/reference -DCMAKE_BUILD_TYPE=Debug
cmake --build build/reference --parallel
ctest --test-dir build/reference --output-on-failure
```

Build the optional visual lab (SDL3 is required only for this target):

```sh
cmake -S evaluator -B build/lab -DVWMINI_BUILD_LAB=ON
cmake --build build/lab --target vwmini_lab --parallel
./build/lab/lab/vwmini_lab
```

The lab defaults to the reference library. To smoke-test a provider through the same
public-API-only seam, select its CMake source directory:

```sh
cmake -S evaluator -B build/lab-provider -DVWMINI_BUILD_LAB=ON \
  -DVWMINI_EVALUATOR_SOURCE_DIR=/path/to/provider-project
cmake --build build/lab-provider --target vwmini_lab --parallel
./build/lab-provider/lab/vwmini_lab
```

If the provider supports a test-disable option, pass its documented option as well. For
the archived Qwen Flash provider used in the evaluation record, the exact configuration was:

```sh
cmake -S evaluator -B build/lab-qwen -DVWMINI_BUILD_LAB=ON \
  -DVWMINI_EVALUATOR_SOURCE_DIR=solutions/qwen38-flash-q5-kl-run-01-repair-02 \
  -DBUILD_TESTING=OFF
```

Lab controls: left click selects an agent or chooses a spawn; right click chooses a
target; `A` adds an agent; `G` updates the selected agent's goal; drag a vertex and
release to rebake; `Space` pauses; `N` steps; `R` resets; middle drag pans; wheel zooms;
`Delete` removes the selected agent. Green lines are freshly queried nav-route previews;
white lines are instantaneous crowd-steering velocities.

## Run conformance against a provider

The provider must be a CMake project defining `vwmini::vwmini` and exposing the
canonical public headers. This runs the four tracks separately:

```sh
./evaluator/conformance/run.sh /path/to/provider-project /tmp/vwmini-conformance
```

For a reference self-check, use `evaluator/reference` as the provider.

Future reference-library, conformance-test, and visual-lab changes must consume the
candidate public API only; they must not access candidate implementation sources.
