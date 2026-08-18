# Model Evaluation Overview

This private index compares completed model workspaces against the VWmini task. Scores
are assigned only after conformance, source review, and architecture review. Time is
recorded as context, not as a quality score for local-vs-hosted runs of the same model.

| Run ID | Model | Runtime / precision | Task revision | Functional tracks | Architecture / quality | Total | State | Detail |
|---|---|---|---|---|---|---:|---|---|
| `ds4f-dwarfstar-aggressive-quant-run-01` | DeepSeek V4 Flash | Local Dwarfstar DS4; aggressive quantization | Pre-planning revision: NFR-009 only | G pass; N/C defects; S pass | Deferred pending repair | — | Fixes requested | [record](ds4f-dwarfstar-aggressive-quant-run-01.md) |

## Comparison policy

- Use identical fixed public API/requirements for runs intended to receive directly
  comparable scores. Record any task/prompt revision prominently.
- Report geometry, navmesh/path, simulation lifecycle, and crowd scores independently.
- Do not score wall-clock time as quality when comparing local and API-hosted versions
  of the same model. Retain it for later same-hardware local-model comparisons.
- Preserve source snapshots, detailed evaluation records, and session metadata under the
  same stable run id.
