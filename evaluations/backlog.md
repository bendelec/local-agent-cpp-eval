# Evaluation Backlog

Planning notes only; this is not a priority order or a commitment to run every model.

## Prepared candidate runs

The following runs have a fresh canonical seed workspace and a provenance stub. They use the
current VWmini task (NFR-009 and NFR-010) and will be evaluated against conformance revision 3
(82 tests) when their first implementation round is complete.

| Run ID | Model | Provider | Status |
|---|---|---|---|
| `gpt56-terra-openai-codex-run-01` | GPT-5.6 terra | OpenAI Codex API subscription; `gpt-5.6-terra` | Seed prepared; not started |
| `qwen38-max-venice-run-01` | Qwen 3.8 Max | Venice API; `qwen-3-8-max` | Seed prepared; not started |
| `ds4-pro-venice-run-01` | DeepSeek V4 Pro | Venice API; `deepseek-v4-pro-0813` | Seed prepared; not started |

Each run will temporarily and without committing override the provider `models.json` listing to
a **128 Ki** (`131072` token) context window, matching the other candidates. Record the effective
request/runtime configuration, task revision, completion artifact, and source fingerprint at
intake. Keep the external candidate workspaces read-only after each model round completes.

## Deferred

- **Ling 3.0 Flash** — a clean canonical seed exists, but the current mainline Lemonade/
  CachyOS llama.cpp package does not yet support its internal architecture. Defer the run until
  routine runtime support arrives; manually building a branch or PR is not warranted for this
  lower-priority model.
