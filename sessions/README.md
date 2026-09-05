# Run Provenance

Keep only sanitized provenance that lets a reader interpret a published result without
re-running it. Use one directory per stable base run id:

```text
sessions/<run-id>/
└── metadata.md     # model/provider, quantization, hardware, task/evaluator revision,
                    # scored source tree, and a sanitized external-artifact reference
```

Do **not** commit raw Pi session exports, transcripts, API credentials, absolute local
paths, usernames, or other sensitive material here. If an original export must be kept,
store it outside this repository and record only a neutral artifact identifier, size, and
checksum when one is available.

VWmini records runtime provenance needed to interpret the delivered library. It does not
evaluate session quality, context curation, compaction, subagent use, or other
meta-cognitive behavior. Those analyses and any detailed session material belong in the
[Reflective Pi](https://github.com/bendelec/reflective-pi) project or a separate restricted
archive.

Do not place evaluator scores here; use `../evaluations/<run-id>.md`. If repairs exist,
the evaluation record must name the exact scored `solutions/<run-id>` or
`solutions/<run-id>-repair-N` tree and fingerprint.
