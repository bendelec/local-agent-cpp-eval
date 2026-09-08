# Evaluation Backlog

Planning notes only; this is not a priority order or a commitment to run every model.

## Prepared candidate runs

The following runs have a fresh canonical seed workspace and a provenance stub. They use the
current VWmini task (NFR-009 and NFR-010) and will be evaluated against conformance revision 3
(82 tests) when their first implementation round is complete.

| Run ID | Model | Provider | Status |
|---|---|---|---|
| `gpt56-terra-openai-codex-run-01` | GPT-5.6 terra | OpenAI Codex API subscription; exact API model id pending | Seed prepared; not started |
| `qwen38-max-venice-run-01` | Qwen 3.8 Max | Venice API; exact catalog id pending | Seed prepared; not started |
| `ds4-pro-venice-run-01` | DeepSeek V4 Pro | Venice API; exact catalog id pending | Seed prepared; not started |

Record each provider’s exact model identifier, request/runtime configuration, task revision,
completion artifact, and source fingerprint at intake. Keep the external candidate workspaces
read-only after each model round completes.

## Deferred

- **Ling 3.0 Flash** — a clean canonical seed exists, but the current mainline Lemonade/
  CachyOS llama.cpp package does not yet support its internal architecture. Defer the run until
  routine runtime support arrives; manually building a branch or PR is not warranted for this
  lower-priority model.
