# Solution Archive

Each child directory is the exact final project copied from one model run. Do not edit
its implementation while evaluating it; preserve it as an archival snapshot that can be
configured and linked through its `vwmini::vwmini` CMake target.

Use a stable lowercase run id, for example:

```text
solutions/ds4f-local-q4-run-01/
```

Build artifacts are ignored repository-wide. Put the matching reviewer report in
`../../evaluations/<run-id>.md` and retained Pi session material in
`../../sessions/<run-id>/`.

Run private conformance without modifying the snapshot:

```sh
../../evaluator/conformance/run.sh . /tmp/vwmini-ds4f-local-q4-run-01
```
