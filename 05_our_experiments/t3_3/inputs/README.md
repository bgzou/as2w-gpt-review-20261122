# t3_3_zerob_sim_20261122 / inputs — 输入清单（大文件不落本地）

本实验的 18 个网格（约 29.4 MB）与 scene 副本已上传到自有服务器，本地只保留 SHA 清单，避免重复占用项目树。

| 输入 | 本地 | 服务器（`experiments/t3_3_zerob_sim_20261122/`） | SHA256 |
|---|---|---|---|
| MuJoCo 场景 | `code/scene_v2.xml`（原始副本，唯一改动：meshdir → `meshes/`） | `code/scene_v2.xml` | `1bcf70ab4322a5bd…` |
| 18 个 STL | **不在本地**（源：`部署参考/as2w-unitreestyle/assets/as2w/meshes/`） | `code/meshes/`（18 文件） | 见 `meshes_sha256.txt` |
| 策略 ONNX | 源：`部署参考/as2w-unitreestyle/sessions/2026-11-20-himloco-dr-10000/data/policy-model-18200.onnx` | `code/policy-model-18200.onnx` | `02352011169cbf1d…` |
| zero-B 输入包 | `../../实机日志反演/results/himloco_zeroB_input_20261122_01.json` | `code/himloco_zeroB_input_20261122_01.json` | `06a183d5956cfa6c…` |
| harness | `code/zerob_sim.py` / `code/zerob_sim_ablate.py` | 同名 | `513b5212…` / `80c1c3c8…` |

`meshes_sha256.txt`：每行 `sha256  filename  bytes`，用于核对服务器副本未被改动。

清理说明：上传用的 `_stage/` 目录与 `_stage.tar.gz`（合计约 42 MB）在核对完服务器解包与运行后删除，登记在 `00_管理与安全/移动与清理登记_20261122.md`。
