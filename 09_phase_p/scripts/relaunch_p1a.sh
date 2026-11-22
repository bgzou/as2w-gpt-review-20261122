#!/usr/bin/env bash
set -u
P=/root/gpufree-data/as2w-cpro/phase_klm_20261122/phase_p_20261122
pkill -f run_one_series || true
pkill -f run_all_p1a || true
sleep 2
rm -f "$P"/results/*.json "$P"/logs/p1a_*.log
python3 - "$P" <<'PY'
import json, pathlib, sys
p = pathlib.Path(sys.argv[1]) / 'params'
for f in sorted(p.glob('p1a_*.json')):
    c = json.loads(f.read_text())
    c['series_every'] = 10          # dt 0.005 -> 0.05 s -> 20 Hz
    f.write_text(json.dumps(c, indent=1) + '\n')
    print('patched', f.name, c['series_every'])
PY
cd "$P" || exit 1
setsid nohup bash code/run_all_p1a.sh > logs/launcher.out 2>&1 < /dev/null &
sleep 10
tail -2 logs/sequence.log
ps aux | grep run_one_series | grep -v grep | wc -l
