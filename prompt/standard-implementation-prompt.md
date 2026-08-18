# VWmini — Implementation Prompt

You are independently implementing **VWmini**, a compact native C++23 library for an
immutable 2D navigation mesh, deterministic point-to-point paths, and local avoidance
between disc agents.

## Workspace boundary

Your current repository contains all task inputs. Work only in it and do not use
external code or unprovided task materials. The harness separately provides development
tools, context management, and subagent prompts.

## Read first

Read these files, in order:

1. `README.md`
2. `docs/requirements/requirements.md`
3. `docs/requirements/mesh.md`
4. `docs/requirements/simulation.md`
5. `docs/requirements/non-functional.md`
6. `docs/architecture/public-api.md`

They are the authoritative contract. The supplied public headers and API specification fix public signatures, defaults, enum
values, error meanings, include paths, and CMake target name.

## Task

Preserve the public declarations in `include/vwmini/`; add only private implementation
details there as needed. Implement the library under `src/`. Add deterministic tests
under `tests/`. Configure CMake so consumers link `vwmini::vwmini` with C++23.
Do not weaken, rewrite, or expand the supplied requirements/API to avoid work.

## Architecture and plan first

Before implementation, create `docs/architecture/architecture.md` for the architecture
you intend to build. Describe concise components/responsibilities, dependency and
ownership/data flow, important seams and interfaces, major decisions/trade-offs, plus
one simple inline diagram. Use it to guide implementation and update it if evidence
changes the design, so it accurately explains the completed code. This is your design:
no internal architecture is prescribed.

Before editing library source, create `docs/architecture/implementation-plan.md`. Divide
the work into ordered work packages and small verifiable slices. For every slice record
its goal, dependencies, expected files/interfaces, verification, and completion status.
Follow the plan through implementation. Revise it explicitly when evidence changes a
decision; do not silently abandon it or repeatedly restart analysis.

Correct functional behavior is necessary but not sufficient. The result will also be
reviewed for architecture, modularity, clarity, maintainability, error handling,
idiomatic C++23, test quality, and unnecessary complexity. Make your own design
choices: no internal architecture is prescribed.

## Working method

- Use the harness-provided subagent prompts and tools where helpful to challenge your
  design, derive tests, review work, simplify code, and check documentation. Treat
  workers as advisors; you remain responsible for the result.
- Use context management deliberately during longer work. Keep the documented work
  package/slice plan current as evidence changes.
- Choose responsibilities and seams that make the code easy to understand, test, and
  change. Avoid frameworks, inheritance, or speculative extension layers unless the
  stated task makes them necessary.
- Use only the C++ standard library. No global mutable state, raw owning pointers,
  mandatory dependencies, console output, threads, callbacks, world management, or
  unrequested features.
- Validate public inputs and return the specified `Result`/`ErrorCode`; never crash,
  terminate, invoke undefined behavior, or silently repair invalid submitted geometry.
- Preserve determinism, tolerance rules, transactional setup behavior, and the fixed
  public API. Keep the library single-threaded as specified.
- After each implementation step, run a simplify/quality pass. Before finishing, run
  the full build and tests, warning-clean compilation, formatting checks if configured,
  and a documentation consistency pass.

## Completion report

State the design/modules you chose and why, work packages/slices completed and any
explicit plan revisions, files changed, exact build/test commands and outcomes, and any
deliberately chosen behavior where the contract permits more than one implementation.
