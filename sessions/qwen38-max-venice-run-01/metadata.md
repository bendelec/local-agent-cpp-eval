# Run Metadata — qwen38-max-venice-run-01

> **Status: repair 01 evaluated; final repair requested.** Keep provenance factual and do not
> record credentials or absolute local paths.

| Field | Value |
|---|---|
| Model | Qwen 3.8 Max |
| Runtime | Venice API. |
| API model identifier | `qwen-3-8-max` |
| Precision | Provider-managed; not applicable. |
| Planned context window | 128 Ki tokens (`131072`) via a temporary, uncommitted `models.json` override. |
| Effective context window | Not captured in the available sanitized run provenance (acknowledged provenance exception). |
| Task revision | Current VWmini task, including NFR-009 and NFR-010; conformance revision 3 (82 tests). |
| Seed package | `../../candidate/` |
| Seed package content fingerprint | `c7307faa082e3c7a1867cee3703aa53427bc3d6695fa2e5c1153926780fbdb4f` |
| Candidate workspace | Completed externally; observed clean Git workspace with 13 logical commits at intake (HEAD `96a071f`); see the sanitized [`intake attestation`](intake.md). |
| Initial source archive | [`../../solutions/qwen38-max-venice-run-01/`](../../solutions/qwen38-max-venice-run-01/), fingerprint `df1985468ee3ecc7d7599d37562a5015c6acf2c45bc61b357f153497f4e63647` (generated build output and `.git` metadata omitted). |
| Prompt | [`../../prompt/standard-implementation-prompt.md`](../../prompt/standard-implementation-prompt.md) |
| Repair prompt 01 | [`../../evaluations/repair-prompts/qwen38-max-venice-run-01-repair-01.md`](../../evaluations/repair-prompts/qwen38-max-venice-run-01-repair-01.md) |
| Repair 01 source archive | [`../../solutions/qwen38-max-venice-run-01-repair-01/`](../../solutions/qwen38-max-venice-run-01-repair-01/), fingerprint `2c90daca2dd11f09015a8bc341cc3530f19a44ec8465d82c03084482208f5ab9` (generated build output and `.git` metadata omitted). |
| Repair 01 evaluation | 81/82 conformance; G 15/15, N 30/30, S 33/33, C 3/4. The sole failure is close reflex-corner following; see the evaluation record. |
| Repair prompt 02 | [`../../evaluations/repair-prompts/qwen38-max-venice-run-01-repair-02.md`](../../evaluations/repair-prompts/qwen38-max-venice-run-01-repair-02.md) |
| Evaluation record | [`../../evaluations/qwen38-max-venice-run-01.md`](../../evaluations/qwen38-max-venice-run-01.md) |
| Intake attestation | [`intake.md`](intake.md), sanitized evaluator observation of workspace Git state. |
| Session export | Pending; retain only a sanitized artifact reference if available. |

Do not place build artifacts or mutable post-run edits in an eventual source archive.
