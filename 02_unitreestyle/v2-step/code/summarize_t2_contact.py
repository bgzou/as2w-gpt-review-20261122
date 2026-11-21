"""Rotate measured simulator forces into the robot frame before averaging."""
import json
import sys
import numpy as np


for path in sys.argv[1:]:
    with np.load(path, allow_pickle=False) as data:
        forces = data["wheel_forces_world_n"][-50:]
        quat = data["root_quaternion_world_xyzw"][-50:]
        quat = quat / np.linalg.norm(quat, axis=-1, keepdims=True)
        vector = -quat[..., None, :3]
        scalar = quat[..., None, 3:4]
        cross = 2 * np.cross(vector, forces)
        body = forces + scalar * cross + np.cross(vector, cross)
        states = data["samples"][-50:]
        summary = {}
        for index, label in enumerate(data["labels"]):
            summary[str(label)] = {
                "lateral_force_body_n": body[:, index, :, 1].mean(0).tolist(),
                "normal_force_world_n": forces[:, index, :, 2].mean(0).tolist(),
                "hip_pd_nm": states[:, index, 24:36:3].mean(0).tolist(),
            }
        print(path, json.dumps(summary, indent=2))
