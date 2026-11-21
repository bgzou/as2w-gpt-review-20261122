"""Score the deterministic flat replay before selecting a training parent."""
import argparse
import json
from pathlib import Path

import numpy as np


def score(path):
    with np.load(path, allow_pickle=False) as replay:
        labels = replay["labels"]
        failures = []
        cases = {}
        for mode, axis, direction in (("forward", 0, 1), ("reverse", 0, -1),
                                      ("left", 1, 1), ("right", 1, -1),
                                      ("yaw_left", 2, 1), ("yaw_right", 2, -1)):
            mask = labels == mode
            if mask.sum() != 6:
                raise ValueError(f"{mode}: expected six replay trials")
            loads = replay["normal_load"][50:, mask]
            clearance = replay["clearance"][50:, mask]
            speed = direction * replay["vel"][50:, mask, axis]
            desired = direction * replay["cmd"][50:, mask, axis]
            if not all(np.isfinite(x).all() for x in (loads, clearance, speed, desired)):
                raise ValueError(f"{mode}: nonfinite replay")
            if not (desired > 0).all():
                raise ValueError(f"{mode}: expected constant nonzero command")
            ratio = (speed / desired).mean(axis=0)
            support = (loads >= 3).all(axis=-1).mean(axis=0)
            airborne = np.zeros_like(loads[0], dtype=bool)
            events = np.zeros_like(loads[0], dtype=int)
            for force, height in zip(loads, clearance, strict=True):
                airborne |= (force < 3) & (height > .015)
                landed = airborne & (force >= 4)
                events += landed
                airborne &= ~landed
            case = cases[mode] = {"min_tracking_ratio": float(ratio.min()),
                                  "min_four_wheel_support": float(support.min()),
                                  "min_events_per_wheel": events.min(axis=0).tolist()}
            if replay["fall"][:, mask].any() or case["min_tracking_ratio"] < .65:
                failures.append(f"{mode}: survival or tracking")
            if axis == 0 and support.min() < .90:
                failures.append(f"{mode}: four-wheel rolling support below 90%")
            if axis != 0 and events.min() < 2:
                failures.append(f"{mode}: fewer than two events on a wheel in a trial")
        return {"qualified": not failures, "failures": failures, "cases": cases}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("replay", type=Path)
    args = parser.parse_args()
    result = score(args.replay)
    output = args.replay.with_name(args.replay.stem + "-gait-gate.json")
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(output)
