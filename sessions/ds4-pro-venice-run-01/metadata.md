# Run Metadata — ds4-pro-venice-run-01

> **Status: prepared; implementation has not started.** This is a provenance stub only.
> Record the effective request configuration, completion artifact, and source archive after the
> run. Do not record credentials or absolute local paths.

| Field | Value |
|---|---|
| Model | DeepSeek V4 Pro |
| Runtime | Venice API. |
| API model identifier | `deepseek-v4-pro-0813` |
| Precision | Provider-managed; not applicable. |
| Planned context window | 128 Ki tokens (`131072`) via a temporary, uncommitted `models.json` override; record the effective setting at intake. |
| Task revision | Current VWmini task, including NFR-009 and NFR-010; conformance revision 3 (82 tests). |
| Seed package | `../../candidate/` |
| Seed package content fingerprint | `c7307faa082e3c7a1867cee3703aa53427bc3d6695fa2e5c1153926780fbdb4f` |
| Candidate workspace | Fresh canonical seed prepared externally; no source archive exists yet. |
| Prompt | [`../../prompt/standard-implementation-prompt.md`](../../prompt/standard-implementation-prompt.md) |
| Evaluation record | To be created at `../../evaluations/ds4-pro-venice-run-01.md` after intake. |
| Session export | Pending; retain only a sanitized artifact reference if available. |

Do not place build artifacts or mutable post-run edits in an eventual source archive.
