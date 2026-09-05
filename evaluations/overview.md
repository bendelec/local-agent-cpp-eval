# Model Evaluation Overview

This index compares completed model workspaces against the VWmini task. Scores are
subjective composite review scores assigned only after conformance, source review, and
architecture review. Time is recorded as context, not as a quality score for
local-vs-hosted runs of the same model.

| Run ID | Model | Runtime / precision | Task revision | Normative conformance | Overall score | State | Detail |
|---|---|---|---|---|---:|---|---|
| `ds4f-dwarfstar-aggressive-quant-run-01` | DeepSeek V4 Flash | Local Dwarfstar DS4; aggressive quantization | Pre-planning revision: NFR-009 only | **76/77**; fails overlap recovery | **40/100** (raw 59; safety cap) | Final after two repairs | [evaluation](ds4f-dwarfstar-aggressive-quant-run-01.md) |
| `ds4f-unquant-run-01` | DeepSeek V4 Flash | Venice hosted; `deepseek-v4-flash-0731` | Current revision: NFR-009 and NFR-010 | **76/77**; follower stalls at reflex corner | **57/100** | Final after two repairs | [evaluation](ds4f-unquant-run-01.md) |
| `qwen38-27b-q8-xl-run-01` | Qwen 3.8 27B | Local Lemonade / llama.cpp; Q8_K_XL GGUF | Current revision: NFR-009 and NFR-010 | **75/77**; finite-extreme containment and follower fail | **68/100** | Final after two repairs | [evaluation](qwen38-27b-q8-xl-run-01.md) |
| `muse-glimmer-ud-q8-xl-run-01` | Muse Glimmer | Local Lemonade; UD-Q8_K_XL | Current revision: NFR-009 and NFR-010 | **75/77**; crossing deadlock and overlap recovery fail | **36/100** | Final after two repairs | [evaluation](muse-glimmer-ud-q8-xl-run-01.md) |
| `laguna-s-2.1-unquant-run-01` | Poolside Laguna S 2.1 | Hosted OpenRouter API; `poolside/laguna-s-2.1` | Current revision: NFR-009 and NFR-010 | **74/77**; finite-extreme, direct-route, containment failures | **63/100** | Final after two repairs | [evaluation](laguna-s-2.1-unquant-run-01.md) |
| `laguna-s-2.1-ds4-run-01` | Poolside Laguna S 2.1 | DS4 run | Current revision: NFR-009 and NFR-010 | **71/77**; direct/contained paths, reflex route, and 3/4 crowd scenarios fail | **51/100** | Final after two repairs | [evaluation](laguna-s-2.1-ds4-run-01.md) |

## Conformance revision 2

The normative suite now has **77** tests: geometry 15, navmesh/path 29, simulation 29,
and crowd 4. Five deterministic navmesh/path tests were added after reproducible defects
showed missing coverage: small valid-triangle containment, finite extreme-coordinate
containment, exact near-equal endpoint handling, multi-cell direct visibility, and
continuous containment on an irregular connected mesh. The reference implementation passes
**77/77**. Every archived final source snapshot above was rebuilt and rerun against this
same revision; scores were reconsidered without changing source archives or repair lineage.

## Comparison policy

- Use identical fixed public API/requirements for runs intended to receive directly
  comparable scores. Record any task/prompt revision prominently.
- Report geometry, navmesh/path, simulation lifecycle, and crowd scores independently.
- Do not score wall-clock time as quality when comparing local and API-hosted versions
  of the same model. Retain it for later same-hardware local-model comparisons.
- Preserve source snapshots, detailed evaluation records, and session metadata under the
  same stable run id.
- Before public release, refine the normative conformance suite when reproducible defects
  expose missing coverage, assign a new suite revision, and rerun every archived final
  snapshot plus the reference. After public release, do not rewrite scores or repair
  lineage retroactively.
