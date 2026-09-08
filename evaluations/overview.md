# Model Evaluation Overview

This index compares completed model workspaces against the VWmini task. Scores are
subjective composite review scores assigned only after conformance, source review, and
architecture review. Time is recorded as context, not as a quality score for
local-vs-hosted runs of the same model.

| Run ID | Model | Runtime / precision | Task revision | Normative conformance | Overall score | State | Detail |
|---|---|---|---|---|---:|---|---|
| `gpt56-terra-openai-codex-run-01` | GPT-5.6 terra | OpenAI Codex API subscription; `gpt-5.6-terra` | Current revision: NFR-009 and NFR-010 | **81/82**; close reflex-corner follower stalls | **40/100** (raw 72; safety cap) | Final after two repairs | [evaluation](gpt56-terra-openai-codex-run-01.md) |
| `ds4f-dwarfstar-aggressive-quant-run-01` | DeepSeek V4 Flash | Local Dwarfstar DS4; aggressive quantization | Pre-planning revision: NFR-009 only | **80/82**; stored-position speed cap and overlap recovery fail | **40/100** (raw 59; safety cap) | Final after two repairs | [evaluation](ds4f-dwarfstar-aggressive-quant-run-01.md) |
| `ds4f-unquant-run-01` | DeepSeek V4 Flash | Venice hosted; `deepseek-v4-flash-0731` | Current revision: NFR-009 and NFR-010 | **80/82**; stored-position speed cap and follower stall fail | **57/100** | Final after two repairs | [evaluation](ds4f-unquant-run-01.md) |
| `qwen38-27b-q8-xl-run-01` | Qwen 3.8 27B | Local Lemonade / llama.cpp; Q8_K_XL GGUF | Current revision: NFR-009 and NFR-010 | **79/82**; finite-extreme, stored-speed, and follower failures | **68/100** | Final after two repairs | [evaluation](qwen38-27b-q8-xl-run-01.md) |
| `qwen38-flash-q5-kl-run-01` | Qwen 3.8 Flash | Local Lemonade / llama.cpp; Q5_K_L | Current revision: NFR-009 and NFR-010 | **78/82**; epsilon-band path/motion tunnelling and stored-speed failures | **75/100** | Final after two repairs | [evaluation](qwen38-flash-q5-kl-run-01.md) |
| `muse-glimmer-ud-q8-xl-run-01` | Muse Glimmer | Local Lemonade; UD-Q8_K_XL | Current revision: NFR-009 and NFR-010 | **78/82**; epsilon-band path/motion and two crowd failures | **36/100** | Final after two repairs | [evaluation](muse-glimmer-ud-q8-xl-run-01.md) |
| `laguna-s-2.1-unquant-run-01` | Poolside Laguna S 2.1 | Hosted OpenRouter API; `poolside/laguna-s-2.1` | Current revision: NFR-009 and NFR-010 | **77/82**; finite-extreme, direct/contained paths, stored-speed, subnormal failures | **63/100** | Final after two repairs | [evaluation](laguna-s-2.1-unquant-run-01.md) |
| `laguna-s-2.1-ds4-run-01` | Poolside Laguna S 2.1 | DS4 run | Current revision: NFR-009 and NFR-010 | **74/82**; direct/contained paths, numeric/lifecycle, and 3/4 crowd failures | **51/100** | Final after two repairs | [evaluation](laguna-s-2.1-ds4-run-01.md) |

## Conformance revision 2

The normative suite now has **77** tests: geometry 15, navmesh/path 29, simulation 29,
and crowd 4. Five deterministic navmesh/path tests were added after reproducible defects
showed missing coverage: small valid-triangle containment, finite extreme-coordinate
containment, exact near-equal endpoint handling, multi-cell direct visibility, and
continuous containment on an irregular connected mesh. The reference implementation passes
**77/77**. Every archived final source snapshot above was rebuilt and rerun against this
same revision; scores were reconsidered without changing source archives or repair lineage.

## Conformance revision 3

The normative suite now has **82** tests: geometry 15, navmesh/path 30, simulation 33,
and crowd 4. Five deterministic public-API regressions were added after final review:

1. epsilon-band endpoint tolerance does not join disconnected components in `find_path`;
2. an agent assigned such a disconnected goal enters `NoPath` and cannot move on `step`;
3. local avoidance cannot tunnel across those disconnected epsilon-band islands;
4. float storage rounding cannot make the observed displacement exceed `max_speed * elapsed`;
   and
5. the smallest finite positive duration leaves every observed state finite and speed-capped.

The reference library was corrected to choose a representable stored endpoint within the
speed budget and passes **82/82**. Every archived final snapshot above was rebuilt and
rerun against this same revision. The score review did not change a numeric total: the new
outcomes either corroborate defects already deducted in the detailed evaluations or are not
material beyond their existing functional deductions. Source archives and repair lineage
remain immutable.

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
