# Run Metadata — qwen38-27b-q8-xl-run-01

| Field | Value |
|---|---|
| Model | Qwen 3.8 27B |
| Runtime | Local Lemonade / llama.cpp (`Qwen3.8-27B-UD-Q8_K_XL`) |
| Precision | Q8_K_XL GGUF, as identified by the configured model/runtime path. |
| Task revision | Current VWmini task, including NFR-009 architecture and NFR-010 implementation-plan deliverables. |
| Initial source snapshot | `../../solutions/qwen38-27b-q8-xl-run-01/` |
| Scored final source | `../../solutions/qwen38-27b-q8-xl-run-01-repair-02/` (see the evaluation record for its tree fingerprint) |
| Evaluation record | `../../evaluations/qwen38-27b-q8-xl-run-01.md` |
| Session export | External operator-retained artifact; final observed size 1,859,871 bytes. It is not published in this repository. |
| Provenance completeness | Initial/final source archives, final completion report, runtime identity, and evaluation are retained. Exact hardware and a session-artifact checksum remain external to this repository. |

The live model printed a completion report before it was written to disk. The operator
asked it to store that report; it then wrote
`docs/architecture/completion-report.md`. The immutable archive was created only after
that final filesystem write and excludes CMake-generated `build*`/`CMakeFiles` outputs.
