# Evaluation Records

Create one reviewer record per model run:

```text
evaluations/<run-id>.md
```

Keep the initial result, a concise intermediate-repair summary, and the final
result in that one record. Source archives preserve exact intermediate code, while the
record explicitly identifies the scored final tree and fingerprint.

Use the standard report structure in
[`evaluator/docs/evaluation-checklist.md`](../evaluator/docs/evaluation-checklist.md).
Record the exact solution commit/tree, toolchain, conformance command/output, G/N/S/C
track results, architecture/code review evidence, score, safety cap if any, and material
defects. Keep reviewer evidence here rather than modifying `../solutions/<run-id>/`.

Every reported tree fingerprint is the SHA-256 value printed by:

```sh
./scripts/tree-fingerprint.py solutions/<scored-run-tree>
```

The script hashes each relative file path and its bytes in sorted order, while excluding
Git metadata and generated CMake/build directories. Record the evaluator revision and
toolchain alongside that fingerprint; the fingerprint alone does not identify either.
