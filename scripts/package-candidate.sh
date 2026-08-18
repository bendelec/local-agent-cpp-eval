#!/usr/bin/env sh
# Create an isolated, candidate-visible task package.
set -eu

if [ "$#" -ne 1 ]; then
    echo "usage: $0 DESTINATION" >&2
    exit 2
fi

source_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)/candidate
destination=$1

if [ -e "$destination" ] && [ "$(find "$destination" -mindepth 1 -maxdepth 1 -print -quit)" ]; then
    echo "destination must be absent or empty: $destination" >&2
    exit 2
fi
mkdir -p "$destination"
cp -R "$source_dir"/. "$destination"/
