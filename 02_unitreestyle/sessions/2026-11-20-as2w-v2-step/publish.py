"""Write manifest.md and publish the staged session archive to GitHub (REST API)."""
import base64
import hashlib
import json
import subprocess
import sys
import urllib.request
from pathlib import Path

STAGE = Path(r"X:\Items\as2w\.stage58\gh-stage-20261120")
REPO = "YuTaoV5/as2w-unitreestyle"
PREFIX = "sessions/2026-11-20-as2w-v2-step"
MESSAGE = """Add the analysis toolchain behind the 2026-11-20 session archive

tools/ now carries every script used to produce the numbers in docs/:

- field/    hardware log analysis (zero-command dither, attitude, yaw/lateral
            slopes, PD torque estimate, lateral-acceleration sign check)
- sim/      sensor-tape torque, static/rigid-hold load vs friction, mass and
            COM negative controls, DR0-vs-DR2 comparison
- parity/   ONNX <-> C++ frame parity (final 7.4e-6 obs / 1.9e-6 targets) and
            the fixture generator
- assets/   USD variants (COM, base mass) and the material inspection probes
- training/ eval drivers and screening chains used for the run matrix
- plotting/ the two scripts that regenerate every figure in plots/

tools/README.md maps each script to its inputs, outputs and the key numbers it
produced, plus a "conclusion -> script" traceability table.
"""


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


rows = []
for p in sorted(STAGE.rglob("*")):
    if p.is_file() and p.name != "manifest.md":
        rows.append((p.relative_to(STAGE).as_posix(), p.stat().st_size, sha256(p)))

manifest = [
    "# 归档清单（2026-11-20 会话）",
    "",
    f"共 {len(rows)} 个文件，合计 {sum(r[1] for r in rows) / 1024 / 1024:.2f} MB。",
    "",
    "| 文件 | 字节 | SHA256 |",
    "|---|---|---|",
]
for rel, size, h in rows:
    manifest.append(f"| `{rel}` | {size} | `{h}` |")
manifest += [
    "",
    "## 未上传（体量原因，已在本机归档）",
    "",
    "| 内容 | 大小 | 位置 / 校验 |",
    "|---|---|---|",
    "| 实机日志全量（field_logs + field_sessions） | 4.86 GB（解包） | `X:/Items/as2w/AS2W-field-logs-20261120/`；"
    "`field_logs_bundle.tar.gz` sha `5c09c4e00b17cfb486e5060f4eb7d50de0152c130e74d5225b92d953e6b65a22`，"
    "`field_sessions.tar.gz` sha `7f3a237c85139ce6b3e0e5f74a61225b9ad6383e3ff20bc30e1fdb657d6a3576` |",
    "| 训练权重与 ONNX | 4.7 MB / 0.8 MB | 训练服务器 `/root/gpufree-data/as2w-runs/<run>/train/`；交付包内 SHA 见合同审计 |",
    "| 传感器级回放带 | 7 MB/份 | 交付包 `replay/sensor_tape.npz` |",
    "| 完整逐轮训练日志（jsonl，各 0.5–1.2 MB） | 9 份 | 本机 `X:/Items/as2w/.stage58/gh2/`；仓库内只保留降采样 CSV |",
    "",
    "## 说明",
    "",
    "- 训练服务器 2026-11-21 11:35 重新可达后补取了全部 8 条臂的逐轮日志，`data/training-*.csv` 由其生成。",
    "- 图表由 `tools/plotting/` 下两个脚本从 `data/` 与 `analysis/` 的可复算数据生成。",
    "- `tools/` 内脚本的输入路径是作者本机的会话工作目录；逻辑与机器无关，改路径即可复跑。",
]
(STAGE / "manifest.md").write_text("\n".join(manifest) + "\n", encoding="utf-8")

files_all = sorted(p for p in STAGE.rglob("*") if p.is_file())
print("staged files:", len(files_all))

inp = "protocol=https\nhost=github.com\n\n"
proc = subprocess.run(["git", "credential", "fill"], input=inp, capture_output=True, text=True)
token = ""
for line in proc.stdout.splitlines():
    if line.startswith("password="):
        token = line[len("password="):]
if not token:
    print("no stored credential")
    sys.exit(1)


def api(method, path, payload=None):
    url = f"https://api.github.com/repos/{REPO}{path}"
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(url, data=data, method=method, headers={
        "Authorization": f"Bearer {token}",
        "Accept": "application/vnd.github+json",
        "X-GitHub-Api-Version": "2022-11-28",
        "Content-Type": "application/json",
        "User-Agent": "as2w-session-publish",
    })
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read().decode())


head = api("GET", "/git/ref/heads/main")["object"]["sha"]
print("main head:", head)
base_tree = api("GET", f"/git/commits/{head}")["tree"]["sha"]

entries = []
for p in files_all:
    rel = p.relative_to(STAGE).as_posix()
    blob = api("POST", "/git/blobs", {
        "content": base64.b64encode(p.read_bytes()).decode(),
        "encoding": "base64",
    })
    entries.append({"path": f"{PREFIX}/{rel}", "mode": "100644", "type": "blob", "sha": blob["sha"]})
print("blobs:", len(entries))

tree = api("POST", "/git/trees", {"base_tree": base_tree, "tree": entries})
new_commit = api("POST", "/git/commits", {"message": MESSAGE, "tree": tree["sha"], "parents": [head]})
api("PATCH", "/git/refs/heads/main", {"sha": new_commit["sha"], "force": False})
print("PUBLISH_OK", new_commit["sha"])
