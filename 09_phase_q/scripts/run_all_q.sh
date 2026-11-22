#!/usr/bin/env bash
set -u
P=/root/gpufree-data/as2w-cpro/phase_klm_20261122/phase_q_20261122
for TAG in q_p1b_lock_base_xy_260s q_p1b_lock_wheel_q_260s ; do
  if [ -s "$P/results/$TAG.json" ]; then continue; fi
  echo "=== $(date -u '+%H:%M:%S') START $TAG" >> "$P/logs/sequence.log"
  bash "$P/code/run_q.sh" "$TAG" >> "$P/logs/sequence.log" 2>&1
  echo "=== $(date -u '+%H:%M:%S') END $TAG rc=$?" >> "$P/logs/sequence.log"
done
echo "=== $(date -u '+%H:%M:%S') ALL DONE" >> "$P/logs/sequence.log"
