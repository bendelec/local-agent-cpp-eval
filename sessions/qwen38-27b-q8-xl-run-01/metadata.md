# Run Metadata — qwen38-27b-q8-xl-run-01

| Field | Value |
|---|---|
| Model | Qwen 3.8 27B |
| Runtime | Local Lemonade / llama.cpp (`Qwen3.8-27B-UD-Q8_K_XL`) |
| Precision | Q8_K_XL GGUF, as identified by the configured model/runtime path. |
| Task revision | Current VWmini task, including NFR-009 architecture and NFR-010 implementation-plan deliverables. |
| Source snapshot | `../../solutions/qwen38-27b-q8-xl-run-01/` |
| Evaluation record | `../../evaluations/qwen38-27b-q8-xl-run-01.md` |
| Session export | Operator-retained Pi session: `/home/cbendele/.pi/agent/sessions/--home-cbendele-Projects-CPlusPlus-vwmini_q38--/2026-08-19T18-41-26-658Z_01a01b54-0682-7a6d-83f2-a96574db41d6.jsonl`; final observed size 1,859,871 bytes. It is retained outside this repository. |
| Provenance completeness | Source archive, final completion report, runtime identity, and evaluation are retained. Exact hardware, complete prompt transcript export, and a session-file checksum remain external to this repository. |

The live model printed a completion report before it was written to disk. The operator
asked it to store that report; it then wrote
`docs/architecture/completion-report.md`. The immutable archive was created only after
that final filesystem write and excludes CMake-generated `build*`/`CMakeFiles` outputs.
