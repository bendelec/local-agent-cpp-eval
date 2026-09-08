#!/usr/bin/env bash
# Configure, build, test and format-check the library. Usage: scripts/check.sh [build-dir]
set -euo pipefail
BUILD_DIR="${1:-build}"
cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Debug > /dev/null
cmake --build "${BUILD_DIR}" --parallel
ctest --test-dir "${BUILD_DIR}" --output-on-failure
if command -v clang-format > /dev/null; then
    mapfile -t sources < <(find include src tests -name '*.hpp' -o -name '*.cpp' | sort)
    if ! clang-format --dry-run -Werror "${sources[@]}"; then
        echo "FORMAT: run 'clang-format -i $(echo "${sources[*]}")'" >&2
        exit 1
    fi
fi
echo "ALL CHECKS PASSED"
