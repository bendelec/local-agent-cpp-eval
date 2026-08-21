# Source Metrics Policy

> **Private evaluator guidance.** These metrics supplement source review. They do not
> replace functional evidence and do not independently determine a score.

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

## Experimental complexity signal

Maximum cyclomatic complexity is not currently a scoring metric. It is sensitive to the
parser/tool version, macro expansion, language features, and whether the tool counts
boolean subexpressions. It can also reward code splitting without making the control flow
more understandable.

For now, reviewers may record a pinned-tool complexity result as audit metadata when a
function appears unusually dense. The preferred trial signal is Clang-Tidy's
`readability-function-cognitive-complexity`, because cognitive complexity is more useful
for finding deeply nested control flow than raw branch count. Record the exact compiler
version, check options, target/compilation database, and whether results include internal
headers. Do not compare values across different tool versions or score them until several
runs establish that the signal is stable and review-useful.

A complexity finding should trigger manual review: explain the function's purpose,
identify whether its control flow is genuinely hard to audit, and assess whether a clear
local simplification exists. It is not itself a defect.
