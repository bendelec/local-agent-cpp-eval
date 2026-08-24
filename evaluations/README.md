# Evaluation Records

Create one private reviewer record per model run:

```text
evaluations/<run-id>.md
```

Keep the initial result, a concise intermediate-repair summary, and the final
result in that one record. Source archives preserve the exact intermediate code.

Use the standard report structure in
[`evaluator/docs/evaluation-checklist.md`](../evaluator/docs/evaluation-checklist.md).
Record the exact solution commit/tree, toolchain, conformance command/output, G/N/S/C
track results, architecture/code review evidence, score, safety cap if any, and material
defects. Keep reviewer evidence here rather than modifying `../solutions/<run-id>/`.
