# Source Metrics Policy

> **Published evaluator guidance.** These metrics supplement source review. They do not
> replace functional evidence or determine a score in isolation.

## Purpose

A compact implementation can be easier to inspect and maintain, but line count and
complexity are poor proxies for quality in isolation. A clear explicit state machine may
rightly be longer than an opaque abstraction; a short function may hide difficult logic;
and a model can reduce a metric by deleting useful validation or tests.

Use the measurements below to make reviews more repeatable and to identify code worth
reading closely. Use them as a tie-breaker only after correctness, contract coverage,
tests, scope, and readability are otherwise substantially equivalent.

## Required qualitative duplication review

For every final source review, explicitly inspect for:

- repeated domain logic that has diverged or could share one clear helper;
- duplicated geometry/numeric predicates with potentially different tolerances;
- bespoke versions of the same validation, route, or agent-state transition; and
- the opposite failure: a premature abstraction that couples unrelated concerns or hides
  a simple local rule.

Record concrete examples and the reviewer decision in the evaluation report. Do not
reward abstraction for its own sake and do not use a clone percentage as a score.

## NCLOC: recorded comparison data

Record **physical non-comment lines of implementation code (NCLOC)** for each final
snapshot:

```sh
./scripts/measure-source.py solutions/<run-id>
```

The script is standard-library Python and scans C/C++ implementation files below
`src/` only. It removes `//` and `/* ... */` comments while preserving string, character,
and raw-string contents, then counts non-blank physical lines. It excludes public
headers, tests, generated output, requirements, documentation, and build files. Report
the command output or its total in the evaluation record.

NCLOC is **not a scored target**. It may support a preference for the smaller solution
only when the compared implementations have equivalent demonstrated behavior, safety,
test discipline, and readability. It must never offset a functional defect, missing test,
or weaker architecture.

## Complexity evidence

Complexity is not an independent point category and never offsets functional defects.
However, the rubric uses the maximum Clang CFG McCabe-complexity and lexical-nesting pair
to establish a ceiling for the **Decomposition and complexity** category. This makes a
reproducible measurement necessary when the candidate has a dense function.

Use a pinned Clang version and record the method, compiler version, function, CFG basic
blocks/edges, the calculation `E - N + 2`, and maximum lexical brace nesting. The
thresholds and resulting category ceiling are defined only in
[`evaluation-rubric.md`](evaluation-rubric.md). Interpret the measurement with the source:
a low number does not prove good decomposition, and a high number should identify the
specific coupled responsibilities that make a function difficult to audit.

Clang-Tidy's `readability-function-cognitive-complexity` may additionally be recorded as
a diagnostic signal. Record its check options, compilation database, and whether internal
headers are included; do not compare its values across tool versions.
