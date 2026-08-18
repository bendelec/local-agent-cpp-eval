# Run Session Archive

Keep original Pi session exports and enough metadata to interpret a model result without
re-running it. Use one directory per stable run id:

```text
sessions/<run-id>/
├── session/        # unmodified exported Pi session material
└── metadata.md     # model/provider, quantization, hardware, prompt/package commit,
                    # start/end time, context/tool/subagent notes, and known deviations
```

Do not place evaluator scores here; use `../evaluations/<run-id>.md`. Large or sensitive
session artifacts may instead be retained outside Git, with `metadata.md` recording the
storage location and checksum.
