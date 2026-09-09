# Evaluation Backlog

Planning notes only; this is not a priority order or a commitment to run every model.

## Run tracking

The following rows track prepared, active, and completed first-round runs. They use the current
VWmini task (NFR-009 and NFR-010) and conformance revision 3 (82 tests).

| Run ID | Model | Provider | Status |
|---|---|---|---|
| `gpt56-terra-openai-codex-run-01` | GPT-5.6 terra | OpenAI Codex API subscription; `gpt-5.6-terra` | Complete: final repair evaluated, 81/82; **40/100 safety-capped** |
| `qwen38-max-venice-run-01` | Qwen 3.8 Max | Venice API; `qwen-3-8-max` | Initial round evaluated (80/82); repair 01 requested |
| `ds4-pro-venice-run-01` | DeepSeek V4 Pro | Venice API; `deepseek-v4-pro-0813` | Seed prepared; not started |

Each run will temporarily and without committing override the provider `models.json` listing to
a **128 Ki** (`131072` token) context window, matching the other candidates. Record the effective
request/runtime configuration, task revision, completion artifact, and source fingerprint at
intake. Archive every completed round immediately. Source archives remain read-only; a repair workspace
may be mutable, but its completed states must be captured as new immutable archives.

## Deferred

- **Ling 3.0 Flash** — a clean canonical seed exists, but the current mainline Lemonade/
  CachyOS llama.cpp package does not yet support its internal architecture. Defer the run until
  routine runtime support arrives; manually building a branch or PR is not warranted for this
  lower-priority model.
