# Run Metadata — muse-glimmer-ud-q8-xl-run-01

> **Status: initial implementation frozen and evaluated.** A repair may follow; this
> metadata identifies the immutable initial source tree rather than a later repair tree.

| Field | Value |
|---|---|
| Model | Muse Glimmer |
| Runtime | Local Lemonade; exact backend/version was not captured. |
| Precision | UD-Q8_K_XL |
| Output-token limit | 32K |
| Hardware | 128 GB unified-memory Strix Halo system |
| Task revision | Current VWmini task, including NFR-009 architecture and NFR-010 implementation-plan deliverables; conformance revision 2. |
| Seed package | `../../candidate/` |
| Seed package content fingerprint | `c7307faa082e3c7a1867cee3703aa53427bc3d6695fa2e5c1153926780fbdb4f` |
| Initial source archive | `../../solutions/muse-glimmer-ud-q8-xl-run-01/` |
| Initial source fingerprint | `f9c8fe14078dbbcf74a551356cc4b3fc22cdb6e02aaac88897eae6ad27992218` |
| Evaluation record | `../../evaluations/muse-glimmer-ud-q8-xl-run-01.md` |
| Completion report | [`model-completion-report.md`](model-completion-report.md), supplied in chat rather than written into the workspace |
| Session export | No sanitized session artifact is retained in this repository. |
| Provenance completeness | Initial source, task package fingerprint, completion report, runtime family, precision, output limit, and evaluation are retained; exact Lemonade build, hardware details, timestamps, and session-artifact checksum were not captured. |

An earlier attempt used Lemonade's default 4K output-token limit. It was discarded before
this run, and its mutable workspace was reset to the seed package; it is neither archived
nor scored. Do not place build artifacts or mutable post-run edits in an eventual repair
source archive.
