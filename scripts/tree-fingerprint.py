#!/usr/bin/env python3
"""Print a stable SHA-256 content fingerprint for an archived project tree."""

from __future__ import annotations

import hashlib
import os
import sys
from pathlib import Path

IGNORED_DIRECTORY_NAMES = {".git", "CMakeFiles", "Testing", "build"}


def is_ignored_directory(name: str) -> bool:
    return name in IGNORED_DIRECTORY_NAMES or name.startswith("build-")


def update_record(digest: hashlib._Hash, tag: bytes, relative_path: Path, content: bytes) -> None:
    digest.update(tag)
    digest.update(relative_path.as_posix().encode("utf-8"))
    digest.update(b"\0")
    digest.update(content)
    digest.update(b"\0")


def tree_fingerprint(root: Path) -> str:
    digest = hashlib.sha256()
    for directory, directories, filenames in os.walk(root, topdown=True, followlinks=False):
        directories[:] = sorted(name for name in directories if not is_ignored_directory(name))
        current = Path(directory)
        for filename in sorted(filenames):
            path = current / filename
            relative_path = path.relative_to(root)
            if path.is_symlink():
                update_record(digest, b"link\0", relative_path, os.readlink(path).encode("utf-8"))
            else:
                update_record(digest, b"file\0", relative_path, path.read_bytes())
    return digest.hexdigest()


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} PROJECT_TREE", file=sys.stderr)
        return 2
    root = Path(sys.argv[1]).resolve()
    if not root.is_dir():
        print(f"not a directory: {root}", file=sys.stderr)
        return 2
    print(tree_fingerprint(root))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
