#!/usr/bin/env bash
set -euo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
root=$(realpath -m -- "$1")
cd "$repo"
sha256sum -c repro/inputs.sha256
mkdir -p -- "$(dirname -- "$root")"
mkdir -- "$root"
mkdir -- "$root/source"
cp -- baseline/* "$root/source/"
