#!/usr/bin/env sh
# Build and run the private public-API conformance tracks against one provider project.
set -eu

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "usage: $0 PROVIDER_SOURCE_DIR [BUILD_DIR]" >&2
    exit 2
fi

provider=$(CDPATH= cd -- "$1" && pwd)
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
build_dir=${2:-"$script_dir/build-under-test"}

cmake -S "$script_dir" -B "$build_dir" \
    -DVWMINI_CONFORMANCE_SOURCE_DIR="$provider" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" --parallel

for label in geometry navmesh simulation crowd; do
    printf '\n=== %s ===\n' "$label"
    ctest --test-dir "$build_dir" --output-on-failure --label-regex "^${label}$"
done
