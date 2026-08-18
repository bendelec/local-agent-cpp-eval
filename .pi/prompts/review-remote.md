---
description: Read-only C++ review using a remote model; never use for candidate runs
argument-hint: "[files or area]"
model: openai-codex/gpt-5.6-terra
thinking: high
---

# Remote Review

Review the requested C++ code and immediate documentation context. Do not edit files.
Report only material findings grouped by severity, with path/line references and a
concrete fix. Check correctness, public-contract compliance, ownership, modularity,
C++23 idioms, tests, warnings, and unnecessary complexity.

## Task

$@
