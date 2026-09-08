#!/usr/bin/env bash
# Format this project's own sources (src/, tests/). Public headers under include/ are
# supplied and stay byte-for-byte unchanged (NFR-002).
set -euo pipefail
find src tests \( -name '*.hpp' -o -name '*.cpp' \) -print0 | xargs -0 clang-format -i
