"""Summarize the two 2026-11-20 RISE63 field recordings without actuating hardware."""

import csv
import json
import re
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SESSIONS = ("20261120-171504_rise63-hold15", "20261120-172823_rise63-hold15")


def read_csv(path, prefix):
    with path.open(encoding="utf-8", errors="replace", newline="") as stream:
        for row in csv.reader(stream):
            if row and row[0] == prefix:
                yield row


def analyze(name):
    directory = ROOT / name
    publisher = (directory / "publisher.log").read_text(encoding="utf-8", errors="replace")
    result = {"session": name, "files": {p.name: p.stat().st_size for p in directory.iterdir() if p.is_file()}}
    result["publisher_rc"] = int(re.search(r"RISE63_REMOTE_RC=(\d+)", publisher).group(1))
    result["events"] = [line for line in publisher.splitlines() if any(
        word in line for word in ("[Passive]", "[FixStand]", "[FixedStand]", "[PolicyHandoff]",
                              "[PolicyWalk]", "FIELD_SAFETY_", "STARTUP FAULT", "DEBUG_LOWCMD_RESULT",
                              "DEBUG_POLICY_RESULT", "FixedStand squat trigger"))]

    states = Counter()
    lowcmd_first = lowcmd_last = None
    lowcmd_kp = {}
    for row in read_csv(directory / "debug_lowcmd.csv", "DEBUG_LOWCMD"):
        state = int(row[3])
        states[state] += 1
        mono = int(row[1])
        if lowcmd_first is None:
            lowcmd_first = mono
        lowcmd_last = mono
        if state not in lowcmd_kp:
            lowcmd_kp[state] = {"leg_kp": float(row[5 + 4]), "leg_kd": float(row[5 + 5]),
                                 "wheel_kd": float(row[5 + 12 * 7 + 5])}
    result["lowcmd"] = {"rows": sum(states.values()), "duration_s":
                         round((lowcmd_last - lowcmd_first) / 1e9, 3) if lowcmd_first else 0,
                         "state_rows": dict(sorted(states.items())), "first_row_gains_by_state": lowcmd_kp}

    phases = Counter()
    first_policy = None
    policy_ticks = []
    wheel_requested_max = [0.0] * 4
    wheel_applied_max = [0.0] * 4
    wheel_at_first = None
    for row in read_csv(directory / "debug_policy.csv", "DEBUG_POLICY"):
        phases[row[2]] += 1
        policy_ticks.append(int(row[3]))
        i = 5 + 3 + 2 + 5 + 3 + 4 + 3
        q = [float(x) for x in row[i:i + 16]]
        i += 16 + 16
        obs_count = int(row[i]); i += 1 + obs_count
        state_count = int(row[i]); i += 1 + state_count
        i += 16 + 16  # raw and wrapped actions
        requested = [[float(x) for x in row[i + 4 * j:i + 4 * j + 4]] for j in range(16)]
        i += 64
        applied = [[float(x) for x in row[i + 4 * j:i + 4 * j + 4]] for j in range(16)]
        if first_policy is None:
            first_policy = {"phase": row[2], "tick": int(row[3]), "sensor_q_config": q,
                            "obs_q_minus_training_reset": [float(x) for x in row[5 + 3 + 2 + 5 + 3 + 4 + 3 + 32 + 1 + 9:5 + 3 + 2 + 5 + 3 + 4 + 3 + 32 + 1 + 21]]}
            wheel_at_first = {"requested": [requested[j][1] for j in range(12, 16)],
                              "applied": [applied[j][1] for j in range(12, 16)]}
        for j in range(4):
            wheel_requested_max[j] = max(wheel_requested_max[j], abs(requested[12 + j][1]))
            wheel_applied_max[j] = max(wheel_applied_max[j], abs(applied[12 + j][1]))
    result["policy"] = {"rows": sum(phases.values()), "phase_rows": dict(phases),
                        "first": first_policy, "first_wheel_dq_targets": wheel_at_first,
                        "max_abs_requested_wheel_dq": wheel_requested_max,
                        "max_abs_applied_wheel_dq": wheel_applied_max,
                        "last_tick": policy_ticks[-1] if policy_ticks else None}

    max_wheel_dq = [0.0] * 4
    max_leg_dq = 0.0
    max_wheel_tau = [0.0] * 4
    first_nonzero_mode = None
    last_valid = None
    samples = 0
    for row in read_csv(directory / "low_capture_fast.log", "LOW_SAMPLE"):
        if len(row) < 14 + 16 * 6:
            continue
        samples += 1
        tick = int(row[4]); last_valid = tick
        if first_nonzero_mode is None and any(int(row[14 + j * 6 + 4]) for j in range(16)):
            first_nonzero_mode = tick
        if first_policy and tick < first_policy["tick"]:
            continue
        for j in range(16):
            dq = abs(float(row[14 + j * 6 + 1]))
            if j < 12:
                max_leg_dq = max(max_leg_dq, dq)
            else:
                max_wheel_dq[j - 12] = max(max_wheel_dq[j - 12], dq)
                max_wheel_tau[j - 12] = max(max_wheel_tau[j - 12], abs(float(row[14 + j * 6 + 3])))
    result["lowstate"] = {"valid_samples": samples, "first_foc_tick": first_nonzero_mode,
                          "last_tick": last_valid, "max_abs_wheel_dq_after_B_hw_FR_FL_RR_RL": max_wheel_dq,
                          "max_abs_leg_dq_after_B_hw": max_leg_dq,
                          "max_abs_wheel_tau_after_B_hw_FR_FL_RR_RL": max_wheel_tau}

    protection = {"rows": 0, "nonzero_motorstate_rows": 0, "nonzero_warning_rows": 0,
                  "first_nonzero_motorstate": None, "max_low_age_ms": 0, "last_complete_row": None}
    for line in (directory / "low_protection.log").read_text(encoding="utf-8", errors="replace").splitlines():
        if not line.startswith("PROTECT,") or "battery_v=" not in line:
            continue
        protection["rows"] += 1
        protection["last_complete_row"] = line
        motor = re.search(r"motorstate_or=0x([0-9a-fA-F]+)", line)
        warn = re.search(r"warning_or=0x([0-9a-fA-F]+)", line)
        age = re.search(r"low_age_ms=(-?\d+)", line)
        if motor and int(motor.group(1), 16):
            protection["nonzero_motorstate_rows"] += 1
            if protection["first_nonzero_motorstate"] is None:
                protection["first_nonzero_motorstate"] = line
        if warn and int(warn.group(1), 16):
            protection["nonzero_warning_rows"] += 1
        if age:
            protection["max_low_age_ms"] = max(protection["max_low_age_ms"], int(age.group(1)))
    result["protection"] = protection
    return result


if __name__ == "__main__":
    report = [analyze(name) for name in SESSIONS]
    (ROOT / "analysis.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
