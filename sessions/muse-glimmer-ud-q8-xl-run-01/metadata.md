# Run Metadata — muse-glimmer-ud-q8-xl-run-01

> **Status: final after two repairs and evaluated.** This metadata records the immutable
> initial, repair-01, and repair-02 source archives; repair-02 is the scored final tree.

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
| Repair prompt 01 | [`../../evaluations/repair-prompts/muse-glimmer-ud-q8-xl-run-01-repair-01.md`](../../evaluations/repair-prompts/muse-glimmer-ud-q8-xl-run-01-repair-01.md) (SHA-256 `36dd9dde6d571962be182993cf6a67a306dbd8a2999e40826ec713057f45e8bb`) |
| Repair 01 source archive | `../../solutions/muse-glimmer-ud-q8-xl-run-01-repair-01/` |
| Repair 01 source fingerprint | `ae7fd95a029b08b8d494e0a17ead6fb01da49912c0ce92e67c1497d7778106d0` |
| Repair 01 completion report | [`model-repair-01-completion-report.md`](model-repair-01-completion-report.md), supplied in chat rather than written into the workspace |
| Repair prompt 02 | [`../../evaluations/repair-prompts/muse-glimmer-ud-q8-xl-run-01-repair-02.md`](../../evaluations/repair-prompts/muse-glimmer-ud-q8-xl-run-01-repair-02.md) (SHA-256 `e3c6cea6e091650f60613013deb2345ed11798e05030afe624a78b9e2410fa00`) |
| Repair 02 source archive | `../../solutions/muse-glimmer-ud-q8-xl-run-01-repair-02/` |
| Repair 02 source fingerprint | `4247eab34dcad49944a55c657705cbcf2850db15a36aa57fe1c269bf82048b5e` |
| Repair 02 completion report | [`model-repair-02-completion-report.md`](model-repair-02-completion-report.md), supplied in chat rather than written into the workspace |
| Completion report | [`model-completion-report.md`](model-completion-report.md), supplied in chat rather than written into the workspace |
| Session export | No sanitized session artifact is retained in this repository. |
| Provenance completeness | Initial and repair source archives, task-package fingerprint, completion reports, runtime family, precision, reported hardware, output limit, and evaluation are retained; exact Lemonade build, exact host details, timestamps, and session-artifact checksum were not captured. |

Do not place build artifacts or mutable post-run edits in an eventual repair source
archive.
