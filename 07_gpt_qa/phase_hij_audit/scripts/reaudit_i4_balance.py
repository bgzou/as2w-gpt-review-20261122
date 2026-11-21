#!/usr/bin/env python3
"""Reaudit the archived I4 wrench with a whole-articulation COM.

Reads the original PhysX result and candidate URDF without modifying them.
The archived result contains settled mean joint angles, not a joint trajectory,
so the whole-body COM reconstruction is a settled-pose approximation.
"""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[3]
BUNDLE = WORKSPACE / "部署参考" / "as2w-sim2real-rootcause-bundle-20261121"
RUN = BUNDLE / "runs" / "as2w_v2_simreal_rootcause_20261119"
DEFAULT_RESULT = RUN / "physx_abl" / "i4_runs" / "base_contacts.json"
DEFAULT_URDF = RUN / "physx_abl" / "urdf_cylwheel" / "as2w-load-v0.urdf"
DEFAULT_OUT = Path(__file__).resolve().parents[1] / "results" / "i4_balance_reaudit.json"


def vector(value):
    return np.fromstring(value or "0 0 0", sep=" ", dtype=float)


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_matrix(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    return c * np.eye(3) + (1 - c) * np.outer(axis, axis) + s * np.array([
        [0, -z, y], [z, 0, -x], [-y, x, 0],
    ])


def quat_xyzw_matrix(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def urdf_center_of_mass(urdf_path, joint_positions):
    root = ET.parse(urdf_path).getroot()
    links = {}
    for node in root.findall("link"):
        inertial = node.find("inertial")
        if inertial is None:
            continue
        origin = inertial.find("origin")
        links[node.attrib["name"]] = (
            float(inertial.find("mass").attrib["value"]),
            vector(origin.attrib.get("xyz") if origin is not None else None),
        )
    children = {}
    all_children = set()
    for node in root.findall("joint"):
        parent = node.find("parent").attrib["link"]
        child = node.find("child").attrib["link"]
        all_children.add(child)
        origin = node.find("origin")
        axis = node.find("axis")
        children.setdefault(parent, []).append((
            node.attrib["name"], node.attrib["type"], child,
            vector(origin.attrib.get("xyz") if origin is not None else None),
            vector(origin.attrib.get("rpy") if origin is not None else None),
            vector(axis.attrib.get("xyz") if axis is not None else "1 0 0"),
        ))
    bases = set(links) - all_children
    if bases != {"base_link"}:
        raise ValueError(f"Unexpected URDF root links: {bases}")
    weighted = np.zeros(3)
    total_mass = 0.0
    fixed_root_weighted = np.zeros(3)
    fixed_root_mass = 0.0
    visited = set()

    def walk(link, pos, rot, in_fixed_root):
        nonlocal weighted, total_mass, fixed_root_weighted, fixed_root_mass
        visited.add(link)
        if link in links:
            mass, inertial_pos = links[link]
            weighted += mass * (pos + rot @ inertial_pos)
            total_mass += mass
            if in_fixed_root:
                fixed_root_weighted += mass * (pos + rot @ inertial_pos)
                fixed_root_mass += mass
        for name, kind, child, offset, rpy, axis in children.get(link, []):
            child_pos = pos + rot @ offset
            child_rot = rot @ rpy_matrix(rpy)
            if kind in ("revolute", "continuous"):
                child_rot = child_rot @ axis_matrix(axis, joint_positions.get(name, 0.0))
            elif kind != "fixed":
                raise ValueError(f"Unsupported joint type: {kind} {name}")
            walk(child, child_pos, child_rot, in_fixed_root and kind == "fixed")

    walk("base_link", np.zeros(3), np.eye(3), True)
    if visited != set(links):
        raise ValueError(f"Unvisited links: {set(links) - visited}")
    return total_mass, weighted / total_mass, fixed_root_weighted / fixed_root_mass, fixed_root_mass


def compact(values):
    return np.round(np.asarray(values, dtype=float), 6).tolist()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", type=Path, default=DEFAULT_RESULT)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    data = json.loads(args.result.read_text(encoding="utf-8"))
    series = data["contact_series"]
    forces = np.asarray(series["Fn_vec"], dtype=float)
    positions = np.asarray(series["contact_pos"], dtype=float)[:, :, 0, :]
    root_com_recorded = np.asarray(series["root_com"], dtype=float)
    root_pose = np.asarray(series["root_pose"], dtype=float)
    if forces.shape != positions.shape or forces.shape[1:] != (4, 3):
        raise ValueError("Unexpected contact series shapes")
    if len(forces) != len(root_pose):
        raise ValueError("Contact and root series length differ")

    joint_positions = {}
    for part, values in (("hip", data["hip_q"]),
                         ("thigh", data["thigh_q"]),
                         ("calf", data["calf_q"])):
        joint_positions.update({f"{leg}_{part}": float(value)
                                for leg, value in zip(("FL", "FR", "RL", "RR"), values)})
    mass, whole_com_base, fixed_root_com_base, fixed_root_mass = urdf_center_of_mass(
        args.urdf, joint_positions)
    rotations = np.stack([quat_xyzw_matrix(pose[3:7]) for pose in root_pose])
    whole_com_world = root_pose[:, :3] + np.einsum("tij,j->ti", rotations, whole_com_base)
    fixed_root_com_world = root_pose[:, :3] + np.einsum(
        "tij,j->ti", rotations, fixed_root_com_base)
    root_com_check = np.linalg.norm(fixed_root_com_world - root_com_recorded, axis=1)
    if abs(mass - data["total_mass_kg"]) > 0.01:
        raise ValueError(f"URDF mass {mass} differs from logged mass {data['total_mass_kg']}")

    # Original implementation: cross of separate time means.
    p_mean, f_mean = positions.mean(0), forces.mean(0)
    root_mean = root_com_recorded.mean(0)
    original = np.cross(p_mean - root_mean, f_mean).sum(0)
    root_per_frame = np.cross(positions - root_com_recorded[:, None, :], forces).sum(1)
    whole_per_frame = np.cross(positions - whole_com_world[:, None, :], forces).sum(1)
    force_per_frame = forces.sum(1)
    whole_origin = np.cross(positions, forces).sum(1) + np.cross(
        whole_com_world, np.array([0., 0., -mass * 9.81]))

    report = {
        "source_result": str(args.result), "source_result_sha256": sha256(args.result),
        "source_urdf": str(args.urdf), "source_urdf_sha256": sha256(args.urdf),
        "limitations": [
            "URDF whole-body COM uses archived settled mean joint angles; per-frame joint angles were not archived.",
            "contact_pos_w may not be force weighted or paired with the net force; individual PhysX contact wrenches were not archived.",
            "The four contact force and position time series are rounded in the archived JSON.",
        ],
        "samples": len(forces), "dt_s": data["dt"],
        "mass_kg": mass, "model_weight_N": mass * 9.81,
        "fixed_root_mass_kg": fixed_root_mass,
        "fixed_root_com_in_base_m": compact(fixed_root_com_base),
        "whole_com_in_base_m": compact(whole_com_base),
        "base_vs_whole_com_world_mean_m": compact((whole_com_world - root_com_recorded).mean(0)),
        "root_com_capture_vs_urdf_fixed_root_com_error_m": {
            "mean": float(root_com_check.mean()), "max": float(root_com_check.max()),
        },
        "sum_force_mean_N": compact(force_per_frame.mean(0)),
        "moment_original_mean_then_cross_Nm": compact(original),
        "moment_original_norm_Nm": float(np.linalg.norm(original)),
        "moment_root_com_per_frame_mean_Nm": compact(root_per_frame.mean(0)),
        "moment_root_com_per_frame_norm_Nm": float(np.linalg.norm(root_per_frame.mean(0))),
        "moment_whole_com_per_frame_mean_Nm": compact(whole_per_frame.mean(0)),
        "moment_whole_com_per_frame_norm_Nm": float(np.linalg.norm(whole_per_frame.mean(0))),
        "moment_whole_com_about_origin_mean_Nm": compact(whole_origin.mean(0)),
        "moment_whole_com_per_frame_norm_p95_Nm": float(np.percentile(np.linalg.norm(whole_per_frame, axis=1), 95)),
        "whole_com_first_half_mean_Nm": compact(whole_per_frame[:len(forces)//2].mean(0)),
        "whole_com_second_half_mean_Nm": compact(whole_per_frame[len(forces)//2:].mean(0)),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
