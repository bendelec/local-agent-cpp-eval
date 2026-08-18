# Evaluation — ds4f-dwarfstar-aggressive-quant-run-01-repair-01

## Run identity

| Field | Value |
|---|---|
| Model/run | DeepSeek V4 Flash, local Dwarfstar DS4; repair of [`ds4f-dwarfstar-aggressive-quant-run-01`](ds4f-dwarfstar-aggressive-quant-run-01.md) |
| Source snapshot | [`../solutions/ds4f-dwarfstar-aggressive-quant-run-01-repair-01/`](../solutions/ds4f-dwarfstar-aggressive-quant-run-01-repair-01/) |
| Source revision | No Git history captured in model workspace. Archive tree fingerprint: `b7b04b3e45aa0e9a9b1b32ef45bdb9338c9935d36aa0b58f24ddec8ce07028df` (inside the archive: `find . -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum`). |
| Session/provenance | Same retained external session as the original run; exact runtime configuration and trace remain pending import. |
| Task revision | Pre-NFR-010 planning revision; no NFR-010 penalty applies. |
| Evaluation state | **Repair incomplete; second feedback issued. Architecture/quality score deferred.** |

The prior archive remains immutable. This is a separately captured repair snapshot.

## Intake

| Check | Result |
|---|---|
| Fresh configure/build through conformance runner | Pass; no warnings reported |
| Public API compatibility | Pass |
| Submission-native tests | 3/3 pass |
| clang-format check | Pass |

## Conformance results

| Track | Result | Change from original |
|---|---:|---|
| G — Geometry | 15/15 pass | Remains passing |
| N — Navmesh and paths | 23/24 pass | Boundary-tolerance direct route fixed; L-shaped route still invalid |
| S — Agent lifecycle and stepping | 28/28 pass | Remains passing |
| C — Local crowd behavior | 2/2 pass | Ordinary supplied crossing now passes; overtaking remains passing |

Command:

```sh
./evaluator/conformance/run.sh \
  solutions/ds4f-dwarfstar-aggressive-quant-run-01-repair-01 \
  /tmp/vwmini-ds4f-dwarfstar-aggressive-quant-run-01-repair-01
```

## Regression status

The repair addressed several original symptoms: boundary-tolerance direct routing,
self-touching outline validation, identical-triangle rejection, waypoint-motion handling,
and the supplied ordinary crossing fixture. The submission added targeted native tests and
updated architecture and completion-report documentation. That completion report
incorrectly claims that all six prior issues are fixed: the L-shaped-route result remains
invalid. Its architecture and completion report also overstate finite dense sampling as a
proof/effectively exact implementation of continuous containment; the limitation is
recorded in the next feedback.

However, `FindPath.BentPathShortensAroundReflexCorner` still fails. Its returned route is:

```text
start → (1, 1) → goal → goal
```

The consecutive duplicate final point violates the path structural contract.

## Additional source-review findings

These findings are recorded as repair feedback, not as a final score:

- Finite sampling is not a proof of continuous segment containment; the implementation
  caps sample count, permitting unobserved narrow holes/exits on long segments.
- An adversarial open-space crossing brings centres below their combined radii because
  decisions compare against prior motion rather than necessarily compatible simultaneous
  selections.
- The revised overlap predicate rejects a valid thin, complete shared-edge adjacency.
- Equal finite endpoints outside the mesh return a successful single-point path before
  endpoint membership is checked.
- `length` overflows for some finite inputs, contradicting its documented guarantee and
  feeding unsafe downstream calculations.

The low-leakage next feedback is at
[`repair-prompts/ds4f-dwarfstar-aggressive-quant-run-01-repair-02.md`](repair-prompts/ds4f-dwarfstar-aggressive-quant-run-01-repair-02.md).

## Final result for repair 01

```text
Build/API gate: PASS
Native tests:   PASS (3/3)
Conformance:    G pass; N fail (1 test); S pass; C pass
Final score:    not assigned — functional gate remains failed
```
