# 09_phase_klm — Phase K/L/M 执行产物（2026-11-22）

对应 `07_gpt_qa/ANSWERS_ROUND_1.md` 与 `04_our_analysis/RPT-018_PhaseKLM_Sim2Real_20261122.md`。

| 目录/文件 | 内容 |
|---|---|
| `p0a/p0a_contact_patch.py` | P0-A harness（MuJoCo 4.0.1；C0/C1/C1f/C3a/C3b/C0M±） |
| `p0a/p0a_contact_patch.json` | P0-A 全部运行结果（含 real anchor、逐候选指标、XML SHA） |
| `p0b/results/*.json` | P0-B 10 次 PhysX 运行（dt 0.0025 控制/扰动/480 s 对照、dt 0.005 480 s 对照、dt 0.01 压力） |
| `p0b/logs/environment_*.txt` | 每次运行的环境、版本与输入 SHA |

**关键数字**
- P0-A：C0 与归档 W_A 一致到 0.01 Nm/0.01°；轮外倾 4.8°（前）/12.1°（后）；3/5 条同轴圆柱仍 `ncon=1`；`Fn·b/2` 上界 1.25/2.05 Nm。
- P0-B：dt 0.0025 两分支 4.014° vs 1.665°（间距 2.35°）；+0.1° 扰动净效应 ≤0.01°；dt 0.01 到 360 s 仍未收敛。

复现：`p0a_contact_patch.py` 需 `input/as2w_scene_base.xml` + `input/meshes/`（STL 不在仓库，按 MANIFEST 回源目录取）+ `input/P55G2.npz`；`p0b` 需 `i_common.py`（SHA `b2bf8557…`）。
