#!/usr/bin/env python3
"""Phase E: Isaac Lab / PhysX current-training asset read-back + static same-contract runs.

Asset under test = the CURRENT training asset chain (residual1220, running 2026-11-20):
  /tmp/as2w-unitreestyle/calibration/candidates/load-v0/as2w-load-v0.urdf
  sha256 7c9740a9... (base mass x1.15, COM -15mm, geometry unchanged from official;
  wheel collisions = FL/FR/RL/RR_foot.STL meshes, NOT simplified cylinders).

Scenarios (explicit closed-loop PD, no RL policy anywhere):
  S  suspended: fix_base=True conversion, P55 T2 target @kp55/kd2, init at measured
     P55S mean pose -> settled applied torque = gravity hold torque (Phase B equiv.)
  G  ground: free base on plane, P55 T2 target @55/2 (compare P55G2 + MuJoCo C0/C1)
  GO ground official: official target [-0.15,0.15,-0.15,0.15,0.6x4,-1.2x4] @200/4
     (compare OG + MuJoCo C0/C1)

Read-back (all scenarios, free-base runs authoritative):
  per-body masses, total mass, wheel body world positions (track width, wheel-center
  y offset vs hip axis), settled contact forces per wheel, applied joint torques.

Run (one scenario per invocation; URDF->USD conversion is cached under usd_cache/):
  OMNI_KIT_ACCEPT_EULA=YES /home/robot/z00945176/isaac_env/venv312/bin/python \
      e1_physx_static.py --scen G|S|GO
"""
import argparse
import csv
import hashlib
import json
import pathlib
import sys

import numpy as np
import torch

P = argparse.ArgumentParser()
P.add_argument('--scen', choices=['S', 'G', 'GO'], required=True)
ARGS = P.parse_args()

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'
URDF = pathlib.Path('/tmp/as2w-unitreestyle/calibration/candidates/load-v0/as2w-load-v0.urdf')
USD_CACHE = RUNS / f'physx_usd_{ARGS.scen.lower()}'

from isaaclab.app import AppLauncher  # noqa: E402

launcher = AppLauncher(headless=True, offscreen_render=False)
app = launcher.app

try:
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.terrains import TerrainImporterCfg
    from isaaclab.utils import configclass
    from isaaclab_physx.physics import PhysxCfg

    import xml.etree.ElementTree as ET

    # type-major (all hips, then thighs, then calves) so target arrays and REAL
    # tau_hip/q_hip ordering (FL/FR/RL/RR) line up
    LEGS = [f'{leg}_{part}' for part in ('hip', 'thigh', 'calf')
            for leg in ('FL', 'FR', 'RL', 'RR')]
    WHEELS = [f'{leg}_foot' for leg in ('FL', 'FR', 'RL', 'RR')]
    # measured real means (V2 corrected model coords) for comparison
    REAL = {
        'S': dict(tag='P55S 25-90',
                  tau_hip=[2.771, -3.162, 2.960, -3.057],
                  q_hip=[-0.0522, 0.0564, -0.0527, 0.0575]),
        'G': dict(tag='P55G2 8-19',
                  tau_hip=[-11.805, 11.695, -15.153, 15.188],
                  q_hip=[0.2142, -0.2155, 0.2779, -0.2756], pitch=-6.13),
        'GO': dict(tag='OG 17-120',
                   tau_hip=[-4.35, 5.61, -5.786, 4.463],
                   q_hip=[-0.1285, 0.1213, -0.1205, 0.1279], pitch=0.65),
    }
    if ARGS.scen == 'GO':
        TGT = {n: v for n, v in zip(LEGS, [-0.15, 0.15, -0.15, 0.15,
                                           0.6, 0.6, 0.6, 0.6,
                                           -1.2, -1.2, -1.2, -1.2])}
        KP, KD = 200.0, 4.0
    else:
        TGT = {n: v for n, v in zip(LEGS, [0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8,
                                           -1.6, -1.6, -1.6, -1.6])}
        KP, KD = 55.0, 2.0

    limits = {j.get('name'): j.find('limit').attrib
              for j in ET.parse(URDF).findall('joint') if j.find('limit') is not None}

    sim_cfg = sim_utils.SimulationCfg(
        dt=0.005, render_interval=4,
        physics=PhysxCfg(gpu_found_lost_aggregate_pairs_capacity=2**27),
        use_newton_actuators=False)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        robot = ArticulationCfg(
            prim_path='{ENV_REGEX_NS}/Robot',
            spawn=sim_utils.UrdfFileCfg(
                asset_path=str(URDF), fix_base=(ARGS.scen == 'S'),
                merge_fixed_joints=True, usd_dir=str(USD_CACHE),
                usd_file_name=f'as2w_{ARGS.scen.lower()}.usd',
                activate_contact_sensors=True, joint_drive=None),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0., 0., 0.55 if ARGS.scen != 'S' else 0.05),
                joint_pos={n: TGT[n] for n in LEGS}),
            actuators={
                'legs': ImplicitActuatorCfg(
                    joint_names_expr=LEGS, stiffness=KP, damping=KD,
                    effort_limit_sim={j: float(limits[j]['effort']) for j in LEGS},
                    velocity_limit_sim={j: float(limits[j]['velocity']) for j in LEGS}),
                'wheels': ImplicitActuatorCfg(
                    joint_names_expr=WHEELS, stiffness=0.0, damping=2.0,
                    effort_limit_sim={j: float(limits[j]['effort']) for j in WHEELS},
                    velocity_limit_sim={j: float(limits[j]['velocity']) for j in WHEELS}),
            })
        contact_forces = ContactSensorCfg(
            prim_path='{ENV_REGEX_NS}/Robot/.*', history_length=1)

    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[2., 2., 1.2], target=[0., 0., 0.3])
    # local static ground (remote GroundPlane.usd is unreachable from this offline host).
    # S = suspended: NO ground at all. A fixed base at z=0.05 still leaves the wheels
    # below z=0, so a plane here loads them with thousands of N and collapses the pose.
    if ARGS.scen != 'S':
        ground_cfg = sim_utils.CuboidCfg(
            size=(6., 6., 0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.35, 0.35)))
        ground_cfg.func('/World/ground', ground_cfg, translation=(0., 0., -0.05))
    scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=3.))
    robot = scene['robot'].view if hasattr(scene['robot'], 'view') else scene['robot']
    robot = scene['robot']
    sim.reset()
    robot.update(sim_cfg.dt)

    names = robot.joint_names
    leg_ids = [names.index(n) for n in LEGS]
    wheel_ids = [names.index(n) for n in WHEELS]
    body_names = robot.body_names
    print('JOINTS', names, flush=True)
    print('BODIES', body_names, flush=True)

    # ---------- runtime read-back: masses ----------
    masses = np.asarray(robot.root_physx_view.get_masses()).reshape(-1)
    print('NMASS', masses.size, 'total_mass_kg',
          round(float(masses.sum()), 3), flush=True)
    # COM read-back (body frame) to verify the URDF inertial origins survived import
    coms_b = robot.data.body_com_pos_b[0].cpu().numpy()
    coms = {b: [round(float(x), 5) for x in coms_b[i]] for i, b in enumerate(body_names)}
    print('COMS_B', json.dumps(coms), flush=True)

    # ---------- settle loop ----------
    dev = robot.device
    tgt_leg = torch.tensor([[TGT[n] for n in LEGS]], device=dev)
    nstep = 12000  # 60 s
    rec_from = int(nstep * 0.8)
    hist = {'tau': [], 'q': [], 'quat': [], 'fn': [], 'fn_vec': [], 'wheel_w': []}
    # explicit drive write (implicit-actuator gains did not reach the PhysX drive)
    robot.write_joint_stiffness_to_sim(KP, joint_ids=leg_ids)
    robot.write_joint_damping_to_sim(KD, joint_ids=leg_ids)
    robot.write_joint_stiffness_to_sim(0.0, joint_ids=wheel_ids)
    robot.write_joint_damping_to_sim(2.0, joint_ids=wheel_ids)
    print('ACTUATORS', {k: (v.stiffness.flatten()[:4].tolist(), v.damping.flatten()[:4].tolist(),
                            v.effort_limit.flatten()[:4].tolist())
                        for k, v in robot.actuators.items()}, flush=True)
    for step in range(nstep):
        if step in (0, 1, 10, 100, 1000, 3000):
            print('DBG', step, 'q', [round(float(x), 3) for x in robot.data.joint_pos[0, leg_ids]],
                  'tau', [round(float(x), 2) for x in robot.data.applied_torque[0, leg_ids]],
                  'z', round(float(robot.data.root_pos_w[0, 2]), 3), flush=True)
        robot.set_joint_position_target(tgt_leg, joint_ids=leg_ids)
        robot.set_joint_velocity_target(torch.zeros(1, len(WHEELS), device=dev),
                                        joint_ids=wheel_ids)
        scene.write_data_to_sim()  # push actuator targets into PhysX (DirectRLEnv does this too)
        sim.step()
        scene.update(sim_cfg.dt)
        if step >= rec_from:
            tau = robot.data.applied_torque[:, leg_ids + wheel_ids]
            q = robot.data.joint_pos[:, leg_ids + wheel_ids]
            quat = robot.data.root_quat_w.clone()
            sensor = scene['contact_forces']
            s_names = list(sensor.body_names)
            wheel_sids = [s_names.index(w) for w in WHEELS]
            fnv = sensor.data.net_forces_w[:, wheel_sids]
            fn = fnv.norm(dim=-1)
            hist['tau'].append(tau[0].cpu().numpy().copy())
            hist['q'].append(q[0].cpu().numpy().copy())
            hist['quat'].append(quat[0].cpu().numpy().copy())
            hist['fn'].append(fn[0].cpu().numpy().copy())
            hist['fn_vec'].append(fnv[0].cpu().numpy().copy())
            if step == rec_from:  # read back wheel world positions once settled
                bw = robot.data.body_pos_w[0].cpu().numpy()
                bq = robot.data.body_quat_w[0].cpu().numpy()
                idx = {b: body_names.index(b) for b in body_names}
                pos = {b: bw[idx[b]] for b in WHEELS + [f'{l}_hip' for l in ('FL', 'FR', 'RL', 'RR')]}
                hist['wheel_w'] = {k: v.tolist() for k, v in pos.items()}
                hist['wheel_q'] = {w: bq[idx[w]].tolist() for w in WHEELS}
    tau = np.stack(hist['tau']).mean(0)
    q = np.stack(hist['q']).mean(0)
    fn = np.stack(hist['fn']).mean(0)
    fn_vec = np.stack(hist['fn_vec']).mean(0)
    quat = np.stack(hist['quat']).mean(0)
    # pitch from mean quat (small angles -> fine for near-settled)
    pitch = float(np.degrees(np.arcsin(np.clip(-2 * (quat[1] * quat[3] - quat[0] * quat[2]), -1, 1))))
    wheel_y = {w: hist['wheel_w'][w][1] for w in WHEELS}
    hip_y = {w: hist['wheel_w'][w.replace('_foot', '_hip')][1] for w in WHEELS}

    # implied hip torque from each wheel's measured contact force and contact point.
    # contact point = wheel body origin + R(quat) * (0, side*0.0546, -0.086)
    # (URDF FL_foot.STL bbox y in [0.0287, 0.0805], z radius 0.086; FR/RR mirrored)
    def quat_rot(qq, v):
        # IsaacLab 28.0 body_quat_w is (x, y, z, w), NOT (w, x, y, z)
        x, y, z, w = qq
        return np.array([
            (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] + 2 * (x * z + w * y) * v[2],
            2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - w * x) * v[2],
            2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] + (1 - 2 * (x * x + y * y)) * v[2]])

    hip_tau_contact = {}
    contact_pt = {}
    if 'wheel_q' in hist:
        for i, w in enumerate(WHEELS):
            side = 1.0 if w[:2] in ('FL', 'RL') else -1.0
            r = (np.asarray(hist['wheel_w'][w])
                 + quat_rot(hist['wheel_q'][w], np.array([0.0, side * 0.0546, -0.086]))
                 - np.asarray(hist['wheel_w'][w.replace('_foot', '_hip')]))
            F = fn_vec[i]
            hip_tau_contact[w] = round(float(r[1] * F[2] - r[2] * F[1]), 3)
            contact_pt[w] = [round(float(x), 4) for x in r]
    out = dict(
        scen=ARGS.scen, urdf=str(URDF),
        urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),
        total_mass_kg=round(float(masses.sum()), 3),
        body_masses={b: round(float(masses[i]), 4) for i, b in enumerate(body_names)},
        body_com_pos_b=coms,
        tau_legs=[round(float(v), 3) for v in tau[:12]],
        q_legs=[round(float(v), 4) for v in q[:12]],
        Fn=[round(float(v), 2) for v in fn],
        Fn_vec=[[round(float(x), 3) for x in v] for v in fn_vec],
        hip_tau_from_contact=hip_tau_contact,
        contact_r_from_hip=contact_pt,
        pitch_deg=round(pitch, 2),
        wheel_world=hist['wheel_w'],
        wheel_quat_xyzw=hist.get('wheel_q'),
        track_width_front_m=round(abs(wheel_y['FL_foot'] - wheel_y['FR_foot']), 4),
        track_width_rear_m=round(abs(wheel_y['RL_foot'] - wheel_y['RR_foot']), 4),
        wheel_center_y_off_m={w: round(wheel_y[w] - hip_y[w], 4) for w in WHEELS},
    )
    print('RESULT ' + json.dumps(out), flush=True)
    (RUNS / 'tables' / f'E_PHYSX_{ARGS.scen}.json').write_text(json.dumps(out, indent=1))
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    try:
        app.close()
    except SystemExit:
        pass
