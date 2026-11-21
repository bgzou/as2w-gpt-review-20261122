"""Select checkpoint from the unchanged input and staged fixed replays."""
from pathlib import Path
import hashlib
import json
import sys

out=Path(sys.argv[1])
labels=sys.argv[2:]
rows=[]
for index,label in enumerate(labels):
    candidate=out/label
    gates=sorted(candidate.glob("*/gate.json"))
    assert len(gates) in (2,4)
    verdicts={f.parent.name:json.loads(f.read_text()) for f in gates}
    fails={name:r["failures"] for name,r in verdicts.items()}
    nfail=sum(map(len,fails.values()))
    loss=sum(
        (6-case["survived"])*5 + case["mean_err"] + (1-case["support"])*.3*(mode in ("forward","reverse","stand"))
        + min(case["loaded_slip_p95_median"] or 0.,2.)*.10
        + min(case.get("joint_high_band_worst_axis_median_deg") or 0.,2.)*.015
        for result in verdicts.values() for mode,case in result["cases"].items()
    )
    meta=json.loads((candidate/"candidate.json").read_text())
    rows.append((nfail,loss,index,label,meta,fails))
nfail,loss,index,label,meta,fails=min(rows)
best=Path(out/label/"checkpoint.pt")
summary={
    "selected":label,"qualified":nfail==0,"selected_sha256":hashlib.sha256(best.read_bytes()).hexdigest(),
    "selection_key":["gate_failures","replay_loss","candidate_order"],
    "candidates":[dict(label=k,origin=m,gate_failure_count=n,loss=round(l,6),failures=f) for n,l,i,k,m,f in rows],
}
temp=out/"selection.tmp"
temp.write_text(json.dumps(summary,indent=2)+"\n")
temp.replace(out/"selection.json")
print(label,flush=True)
