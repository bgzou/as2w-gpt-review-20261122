#!/usr/bin/env bash
# Rewrite the hard-coded original repo root in all scripts to a new package root.
# Usage: bash env/relocate_paths.sh /absolute/path/to/as2w-sim2real-rootcause-bundle-20261121
set -euo pipefail

NEW_ROOT="${1:-}"
if [ -z "$NEW_ROOT" ]; then
  echo "usage: bash env/relocate_paths.sh /absolute/path/to/package-root" >&2
  exit 2
fi
NEW_ROOT="$(cd "$NEW_ROOT" && pwd)"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
SCRIPTS="$PKG/runs/as2w_v2_simreal_rootcause_20261119/scripts"

OLD="/home/robot/z00945176/Rob2Nav"
echo "old root: $OLD"
echo "new root: $NEW_ROOT"
grep -rl "$OLD" "$SCRIPTS" | while read -r f; do
  sed -i "s|$OLD|$NEW_ROOT|g" "$f"
  echo "patched $f"
done
echo "done. verify with: grep -rn '/home/robot' $SCRIPTS || true"
