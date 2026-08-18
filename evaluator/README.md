# Private Evaluator Material

Never distribute this directory to a candidate. It contains the reference library,
private acceptance fixtures, conformance tests, visual diagnostic tool, and scoring
guidance. The harness copies only `candidate/` into each candidate workspace and
injects `prompt/standard-implementation-prompt.md` as text.

## Components

- `reference/` — clean C++23 reference implementation of the canonical public headers,
  plus 65 fast GTest unit/integration tests.
- `conformance/` — black-box, public-API-only headless tests. Its four independently
  labelled tracks are geometry, navmesh/path, simulation lifecycle, and crowd behavior.
- `lab/` — optional SDL3 visual diagnostic/editor, linked only through the public API.
- `docs/evaluation-plan.md` — independent track policy.
- `docs/evaluation-rubric.md` — 100-point source-review rubric.
- `docs/evaluation-checklist.md` — repeatable intake, test, and source-review procedure.
- `docs/benchmark/reference-scenarios.md` — private crowd fixtures.

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
