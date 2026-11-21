"""Persist hourly and stage-change progress records from live process evidence."""
import datetime
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

run=Path(sys.argv[1])
base=Path((run/"base-path.txt").read_text().strip())
out=run/"progress"
out.mkdir(exist_ok=True)

def tail(path,limit=128000):
    if not path.is_file():
        return ""
    with path.open("rb") as stream:
        stream.seek(max(0,path.stat().st_size-limit))
        return stream.read().decode("utf-8","replace")

def process(pidfile):
    try:
        pid=int(pidfile.read_text())
        state=Path(f"/proc/{pid}/stat").read_text().split(") ",1)[1].split()[0]
        args=Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\x00",b" ").decode()
        return {"pid":pid,"live":state!="Z","state":state,"command":args}
    except (FileNotFoundError,ValueError,ProcessLookupError):
        return {"live":False}

def updates(path):
    for line in reversed(tail(path).splitlines()):
        try:
            return int(json.loads(line)["update"])
        except (json.JSONDecodeError,KeyError):
            continue
    return 0

def snapshot():
    current=(run/"current.txt").read_text().strip()
    n=updates(base/"train/reward-components.jsonl")
    stages={}
    for stage in range(1,4):
        d=run/f"stage-{stage}"
        p=d/"selection/selection.json"
        stages[str(stage)]=dict(completed_updates=updates(d/"train/reward-components.jsonl"),
                               selected=json.loads(p.read_text()) if p.is_file() else None)
    gpu=subprocess.run(["nvidia-smi","--query-gpu=memory.used,memory.free,utilization.gpu",
                        "--format=csv,noheader"],capture_output=True,text=True,timeout=15)
    last=tail(base/"train.log")
    def metric(name):
        found=re.findall(re.escape(name)+r":\s*([0-9.eE+-]+)",last)
        return float(found[-1]) if found else None
    data={
        "time":datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
        "current":current,"base_completed_updates":n,
        "base_process":process(base/"train.pid"),
        "manager_process":process(run/"manager.pid"),
        "pack_process":process(run/"deployment-pack.pid"),
        "base_train_log_modified_epoch":(base/"train.log").stat().st_mtime,
        "gpu":gpu.stdout.strip(),"base_metrics":{
            label:metric(label) for label in ("Mean reward","Mean value loss","Mean surrogate loss","Mean action std","Iteration time")},
        "stages":stages,"final_candidate":(run/"final-candidate.txt").read_text().strip()
            if (run/"final-candidate.txt").is_file() else None,
        "archive":(run/"final-deployment-archive.txt").read_text().strip()
            if (run/"final-deployment-archive.txt").is_file() else None,
    }
    return data

last_key=None
last_hour=None
while True:
    try:
        data=snapshot()
        hour=data["time"][:13]
        key=(data["current"],data["base_process"]["live"],data["manager_process"]["live"],
             data["pack_process"]["live"],bool(data["archive"]),data["base_completed_updates"]>=10000)
        (out/"latest.tmp").write_text(json.dumps(data,indent=2)+"\n")
        (out/"latest.tmp").replace(out/"latest.json")
        if key!=last_key or hour!=last_hour:
            filename=data["time"][:19].replace(":","")
            (out/f"{filename}.json").write_text(json.dumps(data,indent=2)+"\n")
            lines=["# AS2-W 训练进度", "",f"记录时间 {data['time']}", "",
                   f"基础训练已完成 {data['base_completed_updates']} / 10000 次更新。",
                   f"当前阶段 {data['current']}", "", "## 累计域随机化", ""]
            for level,record in data["stages"].items():
                selection=record["selected"]
                text=f"阶段 {level} 已完成 {record['completed_updates']} / 600 次更新。"
                if selection:
                    text+=f" 选中 {selection['selected']}，通过验收 {selection['qualified']}。"
                lines.append(text)
            lines+=["", "## 进程与数据", "",json.dumps({
                k:data[k] for k in ("base_process","manager_process","pack_process","gpu","base_metrics")},
                ensure_ascii=False,indent=2)]
            (out/f"{filename}.md").write_text("\n".join(lines)+"\n")
            print(data["time"],data["base_completed_updates"],data["current"],flush=True)
            last_key,last_hour=key,hour
        if data["base_completed_updates"]>=10000 and data["archive"]:
            break
    except Exception as err:
        print(type(err).__name__,str(err),flush=True)
    time.sleep(60)
