#!/usr/bin/env python3
"""Measure physical non-comment implementation lines for a C++ source tree.

This is a descriptive comparison metric, not a correctness or quality score.  The
script scans only implementation-source extensions beneath ``src/`` when given a
project root (or beneath the supplied directory when it has no ``src/`` child).
"""

from __future__ import annotations

import argparse
from pathlib import Path

SOURCE_EXTENSIONS = {".c", ".cc", ".cpp", ".cxx", ".c++", ".h", ".hh", ".hpp", ".hxx"}


def without_comments(text: str) -> str:
    """Return text with C++ line/block comments removed, preserving newlines."""
    output: list[str] = []
    index = 0
    state = "normal"
    raw_end = ""

    while index < len(text):
        char = text[index]
        following = text[index + 1] if index + 1 < len(text) else ""

        if state == "line_comment":
            if char == "\n":
                output.append(char)
                state = "normal"
            index += 1
            continue

        if state == "block_comment":
            if char == "*" and following == "/":
                index += 2
                state = "normal"
            else:
                if char == "\n":
                    output.append(char)
                index += 1
            continue

        if state == "raw_string":
            output.append(char)
            if text.startswith(raw_end, index):
                output.extend(raw_end[1:])
                index += len(raw_end)
                state = "normal"
            else:
                index += 1
            continue

        if state in {"string", "character"}:
            output.append(char)
            if char == "\\" and index + 1 < len(text):
                output.append(text[index + 1])
                index += 2
                continue
            if (state == "string" and char == '"') or (state == "character" and char == "'"):
                state = "normal"
            index += 1
            continue

        # Normal C++ source.
        if char == "/" and following == "/":
            state = "line_comment"
            index += 2
        elif char == "/" and following == "*":
            state = "block_comment"
            index += 2
        elif char == '"':
            output.append(char)
            state = "string"
            index += 1
        elif char == "'":
            output.append(char)
            state = "character"
            index += 1
        elif char == "R" and following == '"':
            delimiter_end = text.find("(", index + 2, index + 19)
            if delimiter_end == -1 or "\n" in text[index + 2 : delimiter_end]:
                output.append(char)
                index += 1
            else:
                delimiter = text[index + 2 : delimiter_end]
                raw_end = ")" + delimiter + '"'
                output.extend(text[index : delimiter_end + 1])
                index = delimiter_end + 1
                state = "raw_string"
        else:
            output.append(char)
            index += 1

    return "".join(output)


def non_comment_lines(path: Path) -> int:
    text = path.read_text(encoding="utf-8")
    return sum(bool(line.strip()) for line in without_comments(text).splitlines())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project_or_source", type=Path)
    args = parser.parse_args()

    root = args.project_or_source
    source_root = root / "src" if (root / "src").is_dir() else root
    files = sorted(path for path in source_root.rglob("*") if path.suffix in SOURCE_EXTENSIONS)
    if not files:
        parser.error(f"no C/C++ source files found under {source_root}")

    counts = [(path, non_comment_lines(path)) for path in files]
    for path, count in counts:
        print(f"{count:6d}  {path.relative_to(root)}")
    print(f"{sum(count for _, count in counts):6d}  total physical NCLOC ({len(counts)} files)")


if __name__ == "__main__":
    main()
