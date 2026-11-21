#!/usr/bin/env python3
"""Phase E audit: PhysX controlled ablations on independent copies (one axis at
a time from the e1 G baseline; formal training assets untouched).

Baseline = e1Physx G (load-v0 URDF, dt 0.005, ground contact_offset 0.005,
default materials, P55 T2 target @55/2, free base).

Axes isolated:
  B_mass_official   base inertial patched back to official 10.08 kg / CoM
                    0.035002 (independent URDF copy under runs/)
  C_fric0.1/0.5/1.0 robot physics_material static/dynamic friction
  D_dt0.002         sim dt 0.002 (MuJoCo parity)
  E_coff0.001/0.02  ground collision contact_offset
  F_compliant_soft/hard  physxMaterial compliant contact spring

Run (one variant per process; app restart per variant):
  OMNI_KIT_ACCEPT_EULA=YES CUDA_VISIBLE_DEVICES=1 \
  /home/robot/z00945176/isaac_env/venv312/bin/python f3_physx_ablation.py --variant A_loadv0_base
  ... then --merge to collect tables/F_E_PHYSX_ABLATION.csv
"""
import argparse
import json
import pathlib

import numpy as np

P = argparse.ArgumentParser()
P.add_argument('--variant', choices=[
    'A_loadv0_base', 'B_mass_official',
    'C_fric0.1', 'C_fric0.5', 'C_fric1.0',
    'D_dt0.002', 'E_coff0.001', 'E_coff0.02',
    'F_compliant_soft', 'F_compliant_hard'], required=True)
P.add_argument('--merge', action='store_true')
ARGS = P.parse_args()

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'
URDF = pathlib.Path('/tmp/as2w-unitreestyle/calibration/candidates/load-v0/as2w-load-v0.urdf')
ABL = RUNS / 'physx_abl'
OUTD = RUNS / 'tables' / 'F_E_PHYSX_ABLATION'

VARIANTS = {
    'A_loadv0_base': dict(),
    'B_mass_official': dict(urdf_patch='official'),
    'C_fric0.1': dict(fric=0.1),
    'C_fric0.5': dict(fric=0.5),
    'C_fric1.0': dict(fric=1.0),
    'D_dt0.002': dict(dt=0.002),
    'E_coff0.001': dict(coff=0.001),
    'E_coff0.02': dict(coff=0.02),
    'F_compliant_soft': dict(stiff=1e4, damp=1e3),
    'F_compliant_hard': dict(stiff=1e6, damp=1e4),
}


def merge():
    OUTD.mkdir(parents=True, exist_ok=True)
    rows = []
    for name in VARIANTS:
        f = OUTD / f'{name}.json'
        if not f.exists():
            print('MISSING', name)
            continue
        r = json.loads(f.read_text())
        rows.append(dict(
            variant=name,
            hipTau='/'.join(f'{x:.2f}' for x in r['tau_hip']),
            hipQ='/'.join(f'{x:.3f}' for x in r['q_hip']),
            thighTau='/'.join(f'{x:.2f}' for x in r['thigh_tau']),
            calfTau='/'.join(f'{x:.2f}' for x in r['calf_tau']),
            pitch_deg=r['pitch'], roll_deg=r['roll'],
            Fn='/'.join(f'{x:.1f}' for x in r['Fn']),
            total_mass_kg=r['total_mass']))
    import csv
    with open(RUNS / 'tables' / 'F_E_PHYSX_ABLATION.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print('wrote F_E_PHYSX_ABLATION.csv', len(rows), 'rows')


if ARGS.merge:
    merge()
    raise SystemExit(0)

KW = VARIANTS[ARGS.variant]

# ---------- independent URDF copies ----------
if KW.get('urdf_patch') == 'official':
    dd = ABL / 'urdf_official'
    dd.mkdir(parents=True, exist_ok=True)
    up = dd / 'as2w-official-mass.urdf'
    txt = URDF.read_text()
    OLD = ('<origin xyz="0.020002000 -0.000756200 0.017521000" rpy="0 0 0" />\n'
           '      <mass value="11.591999999999999" />\n'
           '      <inertia ixx="0.0582990545" ixy="-0.00033419" ixz="-0.0285250025" '
           'iyy="0.1589537475" iyz="0.0003187455" izz="0.17316976" />')
    NEW = ('<origin xyz="0.035002000 -0.000756200 0.017521000" rpy="0 0 0" />\n'
           '      <mass value="10.08" />\n'
           '      <inertia ixx="0.05069483" ixy="-0.0002906" ixz="-0.02480435" '
           'iyy="0.13822065" iyz="0.00027717" izz="0.1505824" />')
    assert OLD in txt, 'base_link inertial block not found'
    up.write_text(txt.replace(OLD, NEW))
    lnk = dd / 'meshes'
    if not lnk.exists():
        lnk.symlink_to(URDF.parent / 'meshes')
    URDF_V = up
else:
    URDF_V = URDF

USD_CACHE = ABL / f'usd_{ARGS.variant}'

from isaaclab.app import AppLauncher  # noqa: E402

launcher = AppLauncher(headless=True, offscreen_render=False)
app = launcher.app

try:
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.utils import configclass
    from isaaclab_physx.physics import PhysxCfg
    try:
        from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg as MatCfg
    except ImportError:
        MatCfg = sim_utils.RigidBodyMaterialCfg

    import xml.etree.ElementTree as ET

    LEGS = [f'{leg}_{part}' for part in ('hip', 'thigh', 'calf')
            for leg in ('FL', 'FR', 'RL', 'RR')]
    WHEELS = [f'{leg}_foot' for leg in ('FL', 'FR', 'RL', 'RR')]
    TGT = {n: v for n, v in zip(LEGS, [0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8,
                                       -1.6, -1.6, -1.6, -1.6])}
    KP, KD = 55.0, 2.0

    limits = {j.get('name'): j.find('limit').attrib
              for j in ET.parse(URDF_V).findall('joint') if j.find('limit') is not None}

    dt = KW.get('dt', 0.005)
    sim_cfg = sim_utils.SimulationCfg(
        dt=dt, render_interval=4,
        physics=PhysxCfg(gpu_found_lost_aggregate_pairs_capacity=2**27),
        use_newton_actuators=False)

    urdf_kw = dict(
        asset_path=str(URDF_V), fix_base=False, merge_fixed_joints=True,
        usd_dir=str(USD_CACHE), usd_file_name=f'as2w_{ARGS.variant}.usd',
        activate_contact_sensors=True, joint_drive=None)
    if 'fric' in KW or 'stiff' in KW:
        mkw = dict(static_friction=KW.get('fric', 0.5),
                   dynamic_friction=KW.get('fric', 0.5))
        if 'stiff' in KW:
            mkw.update(compliant_contact_stiffness=KW['stiff'],
                       compliant_contact_damping=KW['damp'])
        urdf_kw['physics_material'] = MatCfg(**mkw)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        robot = ArticulationCfg(
            prim_path='{ENV_REGEX_NS}/Robot',
            spawn=sim_utils.UrdfFileCfg(**urdf_kw),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0., 0., 0.55), joint_pos={n: TGT[n] for n in LEGS}),
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
    coff = KW.get('coff', 0.005)
    ground_cfg = sim_utils.CuboidCfg(
        size=(6., 6., 0.1),
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=coff, rest_offset=0.0),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.35, 0.35)))
    ground_cfg.func('/World/ground', ground_cfg, translation=(0., 0., -0.05))
    scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=3.))
    robot = scene['robot']
    sim.reset()
    robot.update(sim_cfg.dt)

    names = robot.joint_names
    leg_ids = [names.index(n) for n in LEGS]
    wheel_ids = [names.index(n) for n in WHEELS]
    body_names = robot.body_names

    masses = np.asarray(robot.root_physx_view.get_masses()).reshape(-1)

    dev = robot.device
    import torch
    tgt_leg = torch.tensor([[TGT[n] for n in LEGS]], device=dev)
    nstep = int(12000 * (0.005 / dt))  # same 60 s physical time
    rec_from = int(nstep * 0.8)
    hist = {'tau': [], 'q': [], 'quat': [], 'fn': []}
    robot.write_joint_stiffness_to_sim(KP, joint_ids=leg_ids)
    robot.write_joint_damping_to_sim(KD, joint_ids=leg_ids)
    robot.write_joint_stiffness_to_sim(0.0, joint_ids=wheel_ids)
    robot.write_joint_damping_to_sim(2.0, joint_ids=wheel_ids)
    for step in range(nstep):
        robot.set_joint_position_target(tgt_leg, joint_ids=leg_ids)
        robot.set_joint_velocity_target(torch.zeros(1, len(WHEELS), device=dev),
                                        joint_ids=wheel_ids)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_cfg.dt)
        if step >= rec_from:
            tau = robot.data.applied_torque[:, leg_ids + wheel_ids]
            q = robot.data.joint_pos[:, leg_ids + wheel_ids]
            hist['tau'].append(tau[0].cpu().numpy().copy())
            hist['q'].append(q[0].cpu().numpy().copy())
            hist['quat'].append(robot.data.root_quat_w[0].cpu().numpy().copy())
            sensor = scene['contact_forces']
            s_names = list(sensor.body_names)
            wheel_sids = [s_names.index(w) for w in WHEELS]
            hist['fn'].append(sensor.data.net_forces_w[0, wheel_sids].cpu().numpy().copy())
    tau = np.stack(hist['tau']).mean(0)
    q = np.stack(hist['q']).mean(0)
    fn = np.stack(hist['fn']).mean(0)
    quat = np.stack(hist['quat']).mean(0)
    # IsaacLab quat is (x, y, z, w)
    pitch = float(np.degrees(np.arcsin(np.clip(-2 * (quat[1] * quat[3] - quat[0] * quat[2]), -1, 1))))
    roll = float(np.degrees(np.arctan2(2 * (quat[0] * quat[1] + quat[3] * quat[2]),
                                       1 - 2 * (quat[1] ** 2 + quat[2] ** 2))))
    ta = tau[:12]
    out = dict(
        variant=ARGS.variant,
        kwargs={k: v for k, v in KW.items()},
        dt=dt, nsec=round(nstep * dt, 1),
        urdf=str(URDF_V),
        total_mass=round(float(masses.sum()), 3),
        # LEGS is type-major: indices 0-3 hips, 4-7 thighs, 8-11 calves
        tau_hip=[round(float(v), 3) for v in ta[0:4]],
        q_hip=[round(float(v), 4) for v in q[0:4]],
        thigh_tau=[round(float(v), 3) for v in ta[4:8]],
        calf_tau=[round(float(v), 3) for v in ta[8:12]],
        pitch=round(pitch, 2), roll=round(roll, 2),
        Fn=[round(float(np.linalg.norm(v)), 1) for v in fn],
        Fn_vec=[[round(float(x), 2) for x in v] for v in fn],
    )
    print('RESULT ' + json.dumps(out), flush=True)
    OUTD.mkdir(parents=True, exist_ok=True)
    (OUTD / f'{ARGS.variant}.json').write_text(json.dumps(out, indent=1))
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    try:
        app.close()
    except SystemExit:
        pass
