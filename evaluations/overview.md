# Model Evaluation Overview

This private index compares completed model workspaces against the VWmini task. Scores
are assigned only after conformance, source review, and architecture review. Time is
recorded as context, not as a quality score for local-vs-hosted runs of the same model.

| Run ID | Model | Runtime / precision | Task revision | Functional tracks | Architecture / quality | Total | State | Detail |
|---|---|---|---|---|---|---:|---|---|
| `ds4f-dwarfstar-aggressive-quant-run-01` | DeepSeek V4 Flash | Local Dwarfstar DS4; aggressive quantization | Pre-planning revision: NFR-009 only | Public conformance passes; safety/source defects remain | **40/100 cap** | Raw 64/100 | Final after two repairs | [original](ds4f-dwarfstar-aggressive-quant-run-01.md), [repair 01](ds4f-dwarfstar-aggressive-quant-run-01-repair-01.md), [final repair](ds4f-dwarfstar-aggressive-quant-run-01-repair-02.md) |
| `qwen38-27b-q8-xl-run-01` | Qwen 3.8 27B | Local Lemonade / llama.cpp; Q8_K_XL GGUF | Current revision: NFR-009 and NFR-010 | **69/69 public conformance; 72/72 native and sanitized** | **13/15** | **97/100** | Final, first attempt | [evaluation](qwen38-27b-q8-xl-run-01.md) |

## Comparison policy

- Use identical fixed public API/requirements for runs intended to receive directly
  comparable scores. Record any task/prompt revision prominently.
- Report geometry, navmesh/path, simulation lifecycle, and crowd scores independently.
- Do not score wall-clock time as quality when comparing local and API-hosted versions
  of the same model. Retain it for later same-hardware local-model comparisons.
- Preserve source snapshots, detailed evaluation records, and session metadata under the
  same stable run id.
