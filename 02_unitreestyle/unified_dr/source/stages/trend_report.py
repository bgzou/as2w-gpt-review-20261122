"""Snapshot PPO curves and task-specific reward components without training changes."""
import argparse
import json
import math
from pathlib import Path
import statistics

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

p=argparse.ArgumentParser()
p.add_argument("train",type=Path)
p.add_argument("out",type=Path)
args=p.parse_args()
reader=EventAccumulator(str(args.train),size_guidance={"scalars":0}).Reload()
curves={}
for tag in reader.Tags()["scalars"]:
    events=reader.Scalars(tag)
    if not events: continue
    blocks=[]
    for start in range(0,len(events),100):
        block=events[start:start+100]
        values=[v.value for v in block]
        blocks.append(dict(first_step=block[0].step,last_step=block[-1].step,
                           mean=statistics.fmean(values),minimum=min(values),maximum=max(values)))
    curves[tag]=dict(count=len(events),last_step=events[-1].step,last_value=events[-1].value,
                     finite=all(math.isfinite(v.value) for v in events),blocks=blocks)
lines=(args.train/"reward-components.jsonl").read_text().splitlines()
records=[json.loads(s) for s in lines if s.strip()]
components={}
if records:
    for key in records[0]:
        if key=="update": continue
        components[key]=[dict(first_update=part[0]["update"],last_update=part[-1]["update"],
                               mean=statistics.fmean(row[key] for row in part))
                         for start in range(0,len(records),100) if (part:=records[start:start+100])]
result=dict(train=str(args.train),tensorboard=curves,reward_components=components,
            completed_updates=records[-1]["update"] if records else None)
args.out.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
for tag,data in curves.items():
    print(tag,data["last_step"],round(data["last_value"],6))
