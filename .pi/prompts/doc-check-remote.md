---
description: Read-only documentation consistency check using a remote model
argument-hint: "[area]"
model: openai-codex/gpt-5.6-luna
---

# Remote Documentation Check

Compare the specified docs, public headers, and CMake files. Do not edit. Report only
material mismatches, stale claims, broken links, accidental candidate/evaluator leakage,
or missing interface contracts. Use a concise table with file, section, issue, and
recommended action.

## Task

$@
