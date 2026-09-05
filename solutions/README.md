# Solution Archive

Each child directory is an immutable project tree copied from a model run. A lineage
may retain an initial output and separately named repair snapshots; the matching
`evaluations/<run-id>.md` identifies the exact scored final tree and fingerprint. Do not
edit an archived implementation while evaluating it. Preserve every snapshot as an
archival source tree that can be configured and linked through its `vwmini::vwmini` CMake
target.

Use a stable lowercase run id, for example:

```text
solutions/ds4f-local-q4-run-01/
```

Build artifacts are ignored repository-wide. Put the matching reviewer report in
`../../evaluations/<run-id>.md` and sanitized run provenance in
`../../sessions/<run-id>/`. Keep raw Pi session material in an external restricted archive
or the Reflective Pi project, not in this repository.

Run published conformance without modifying the snapshot:

```sh
../../evaluator/conformance/run.sh . /tmp/vwmini-ds4f-local-q4-run-01
```
