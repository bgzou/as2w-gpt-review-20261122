import json
import sys
from pathlib import Path
stage, run = map(Path,sys.argv[1:])
from paired import compare
eligible = []
for p in stage.glob("*/gate.json"):
    gate = compare(run/"paired1220",p.parent)
    if gate["gait_preserved"]:
        cases = gate["case_metrics"].values()
        ratios = [c["phase_candidate"]["measured_residual_rms_deg"] / c["phase_baseline"]["measured_residual_rms_deg"] for c in cases if c.get("phase_candidate") and c.get("phase_baseline")]
        if len(ratios) != 4:
            continue
        active=("left","right","yaw_left","yaw_right","mixed")
        def spikes(name):
            return sum(gate["wheel_load_"+name][k]["short_reversals"] for k in active)
        eligible.append((sum(ratios)/len(ratios),spikes("candidate"),str(p.parent/"best.pt"),spikes("baseline")))
if not eligible:
    sys.exit("No candidate preserved the baseline gait. See individual gate.json files.")
ratio, spikes, checkpoint, original = min(eligible)
record = dict(mean_phase_residual_ratio=ratio,spikes=spikes,checkpoint=checkpoint)
(stage/"selection.json").write_text(json.dumps(record,indent=2))
best = run/"validated-best.json"
if ratio<1 and spikes<original and (not best.exists() or ratio<json.loads(best.read_text())["mean_phase_residual_ratio"]):
    best.write_text(json.dumps(record,indent=2))
print(checkpoint)
