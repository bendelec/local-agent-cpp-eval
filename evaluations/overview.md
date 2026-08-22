# Model Evaluation Overview

This private index compares completed model workspaces against the VWmini task. Scores
are assigned only after conformance, source review, and architecture review. Time is
recorded as context, not as a quality score for local-vs-hosted runs of the same model.

| Run ID | Model | Runtime / precision | Task revision | Normative conformance | Overall score | State | Detail |
|---|---|---|---|---|---:|---|---|
| `ds4f-dwarfstar-aggressive-quant-run-01` | DeepSeek V4 Flash | Local Dwarfstar DS4; aggressive quantization | Pre-planning revision: NFR-009 only | **71/72** expanded normative conformance; fails overlap recovery | **40/100** (raw 62; safety cap) | Final after two repairs | [original](ds4f-dwarfstar-aggressive-quant-run-01.md), [repair 01](ds4f-dwarfstar-aggressive-quant-run-01-repair-01.md), [final repair](ds4f-dwarfstar-aggressive-quant-run-01-repair-02.md) |
| `qwen38-27b-q8-xl-run-01` | Qwen 3.8 27B | Local Lemonade / llama.cpp; Q8_K_XL GGUF | Current revision: NFR-009 and NFR-010 | Re-evaluation pending replacement repair 02 | — | Second repair replay in progress | [first attempt](qwen38-27b-q8-xl-run-01.md); [repair-01 baseline](../solutions/qwen38-27b-q8-xl-run-01-repair-01/); [superseded repair-02](../solutions/qwen38-27b-q8-xl-run-01-repair-02-superseded-01/); [replacement prompt](repair-prompts/qwen38-27b-q8-xl-run-01-repair-02.md) |

## Comparison policy

- Use identical fixed public API/requirements for runs intended to receive directly
  comparable scores. Record any task/prompt revision prominently.
- Report geometry, navmesh/path, simulation lifecycle, and crowd scores independently.
- Do not score wall-clock time as quality when comparing local and API-hosted versions
  of the same model. Retain it for later same-hardware local-model comparisons.
- Preserve source snapshots, detailed evaluation records, and session metadata under the
  same stable run id.
- Before public release, newly discovered reproducible defects may refine normative
  conformance. If a repair prompt did not include such a defect, replace (rather than add
  to) that model's affected repair attempt from its prior archived baseline; retain the
  superseded archive and label the replay. Qwen's replacement repair 02 starts from its
  repair-01 archive and supersedes (but does not delete) the first repair-02 archive.
  After public release, do not rewrite scores or repair lineage retroactively.
