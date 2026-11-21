#!/usr/bin/env python3
"""G3: IsaacLab/PhysX wheel-collision-primitive x scenario x init-condition matrix.

Companion to g3_mujoco_matrix.py.  18 runs:
  scenario {S suspended, G ground-P55, GO ground-official}
  x primitive {cyl, mesh} x init {real, nearflat, crouch}.

Unified conditions (vs the earlier load-v0 baseline): base inertial patched to
the DELIVERY model (mass 10.08 kg, CoM 0.035002) so both engines share masses;
leg/foot masses unchanged; wheel radius 0.086, width 0.052, centre offset
+/-0.0546; robot material friction 1.0; ground contact_offset 0.005; dt 0.005;
gains G kp55/kd2, GO kp200/kd4 (kd4 matches e1); same targets; gravity -9.81.

Primitives:
  mesh = urdf_meshwheel/as2w-load-v0.urdf  (foot *_foot.STL collision)
  cyl  = urdf_cylwheel/as2w-load-v0.urdf   (foot collision replaced by
         <cylinder radius="0.086" length="0.052"/> at y=+/-0.0546, axis along y)

PhysX does not expose per-contact point counts through ContactSensor (net force
only), so n_contacts_per_wheel is reported as -1 / UNKNOWN for PhysX rows.

Usage:
  # driver (runs all 18 sequentially, one app process each, merges tables):
  OMNI_KIT_ACCEPT_EULA=YES CUDA_VISIBLE_DEVICES=1 \
  LD_LIBRARY_PATH=/home/robot/miniconda3/lib \
  /home/robot/z00945176/isaac_env/venv312/bin/python g3_physx_matrix.py
  # single worker (used internally):
  ... g3_physx_matrix.py --worker --scen G --prim mesh --init real
"""
import argparse
import csv
import json
import os
import pathlib
import subprocess
import sys

import numpy as np

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'
ABL = RUNS / 'physx_abl'
OUTD = ABL / 'g3_runs'
LOGD = ABL / 'g3_logs'
NPZ = RUNS / 'qc' / 'a1_timelines' / 'P55G2.npz'
TABLE = RUNS / 'tables' / 'G3_ENGINE_PRIMITIVE_MATRIX'

LEGS = [f'{leg}_{part}' for part in ('hip', 'thigh', 'calf')
        for leg in ('FL', 'FR', 'RL', 'RR')]
WHEELS = [f'{leg}_foot' for leg in ('FL', 'FR', 'RL', 'RR')]
R_WHEEL = 0.086


def real_init_q():
    z = np.load(NPZ)
    t, q = z['t'], z['q']
    m = (t >= 8.0) & (t <= 19.0)
    return [round(float(x), 6) for x in q[m].mean(0)]


INITS = {
    'real': real_init_q(),
    'nearflat': [0.03] * 4 + [0.5] * 4 + [-1.2] * 4,
    'crouch': [0.21] * 4 + [1.2] * 4 + [-2.4] * 4,
}
SCEN = {
    'G': dict(tgt=[0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8, -1.6, -1.6, -1.6, -1.6],
              kp=55.0, kd=2.0, ground=True),
    'GO': dict(tgt=[-0.15, 0.15, -0.15, 0.15, 0.6, 0.6, 0.6, 0.6,
                    -1.2, -1.2, -1.2, -1.2], kp=200.0, kd=4.0, ground=True),
    'S': dict(tgt=[0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8, -1.6, -1.6, -1.6, -1.6],
              kp=55.0, kd=2.0, ground=False),
}
URDF = {'mesh': ABL / 'urdf_meshwheel' / 'as2w-load-v0.urdf',
        'cyl': ABL / 'urdf_cylwheel' / 'as2w-load-v0.urdf'}


def worker(scen, prim, init_name):
    import torch
    from isaaclab.app import AppLauncher
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

        cfg = SCEN[scen]
        TGT = {n: v for n, v in zip(LEGS, cfg['tgt'])}
        KP, KD = cfg['kp'], cfg['kd']
        q0 = INITS[init_name]
        urdf = URDF[prim]
        limits = {j.get('name'): j.find('limit').attrib
                  for j in ET.parse(urdf).findall('joint')
                  if j.find('limit') is not None}
        usd_cache = ABL / 'g3_usd' / f'{prim}_{scen}_{init_name}'
        tag = f'{scen}_{prim}_{init_name}'

        sim_cfg = sim_utils.SimulationCfg(
            dt=0.005, render_interval=4,
            physics=PhysxCfg(gpu_found_lost_aggregate_pairs_capacity=2**27),
            use_newton_actuators=False)

        @configclass
        class SceneCfg(InteractiveSceneCfg):
            robot = ArticulationCfg(
                prim_path='{ENV_REGEX_NS}/Robot',
                spawn=sim_utils.UrdfFileCfg(
                    asset_path=str(urdf), fix_base=(scen == 'S'),
                    merge_fixed_joints=True, usd_dir=str(usd_cache),
                    usd_file_name=f'as2w_{tag}.usd',
                    activate_contact_sensors=True, joint_drive=None,
                    physics_material=MatCfg(static_friction=1.0,
                                            dynamic_friction=1.0)),
                init_state=ArticulationCfg.InitialStateCfg(
                    pos=(0., 0., 0.05 if scen == 'S' else 0.55),
                    joint_pos={n: q0[i] for i, n in enumerate(LEGS)}),
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
        if cfg['ground']:
            ground_cfg = sim_utils.CuboidCfg(
                size=(6., 6., 0.1),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    contact_offset=0.005, rest_offset=0.0),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    kinematic_enabled=True, disable_gravity=True),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.35, 0.35, 0.35)))
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
        print('TOTAL_MASS', round(float(masses.sum()), 3), flush=True)

        dev = robot.device
        # NOTE: ArticulationCfg.InitialStateCfg.joint_pos was observed NOT to be
        # applied at reset in this IsaacLab build (legs started straight), so we
        # write the initial joint state explicitly here.
        q0_t = torch.tensor([[q0[i] for i in range(12)]], device=dev)
        robot.write_joint_position_to_sim_index(position=q0_t, joint_ids=leg_ids)
        robot.write_joint_velocity_to_sim_index(
            velocity=torch.zeros_like(q0_t), joint_ids=leg_ids)
        robot.write_joint_position_to_sim_index(
            position=torch.zeros(1, len(WHEELS), device=dev), joint_ids=wheel_ids)
        robot.update(sim_cfg.dt)
        print('INIT_Q', [round(float(x), 4) for x in
                         robot.data.joint_pos[0, leg_ids].cpu().numpy()], flush=True)
        # place wheels ~2 mm above ground (ground scenarios, free base)
        if cfg['ground']:
            bw = robot.data.body_pos_w[0].cpu().numpy()
            wz = [float(bw[body_names.index(w)][2]) for w in WHEELS]
            lowest = min(wz) - R_WHEEL
            rp = robot.data.root_pose_w.clone()  # (x,y,z,qx,qy,qz,qw)
            rp[0, 2] = rp[0, 2] + (0.002 - lowest)
            robot.write_root_pose_to_sim_index(root_pose=rp)
            robot.update(sim_cfg.dt)
            print('START_Z', round(float(rp[0, 2].item()), 4), 'lowest_before',
                  round(lowest, 4), flush=True)

        tgt_leg = torch.tensor([[TGT[n] for n in LEGS]], device=dev)
        nstep = 12000  # 60 s
        rec_from = int(nstep * 0.8)
        hist = {'tau': [], 'q': [], 'quat': [], 'fn': [], 'fn_vec': [],
                'dqmax': []}
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
            dqv = robot.data.joint_vel[0, leg_ids]
            hist['dqmax'].append(float(dqv.abs().max().item()))
            if step >= rec_from:
                tau = robot.data.applied_torque[:, leg_ids + wheel_ids]
                q = robot.data.joint_pos[:, leg_ids + wheel_ids]
                hist['tau'].append(tau[0].cpu().numpy().copy())
                hist['q'].append(q[0].cpu().numpy().copy())
                hist['quat'].append(robot.data.root_quat_w[0].cpu().numpy().copy())
                sensor = scene['contact_forces']
                s_names = list(sensor.body_names)
                wheel_sids = [s_names.index(w) for w in WHEELS]
                fnv = sensor.data.net_forces_w[0, wheel_sids]
                hist['fn'].append(fnv.norm(dim=-1).cpu().numpy().copy())
                hist['fn_vec'].append(fnv.cpu().numpy().copy())
        tau = np.stack(hist['tau']).mean(0)
        q = np.stack(hist['q']).mean(0)
        fn = np.stack(hist['fn']).mean(0)
        fn_vec = np.stack(hist['fn_vec']).mean(0)
        quat = np.stack(hist['quat']).mean(0)  # (x,y,z,w)
        pitch = float(np.degrees(np.arcsin(np.clip(
            -2 * (quat[1] * quat[3] - quat[0] * quat[2]), -1, 1))))
        dqmax = np.array(hist['dqmax'])
        over = np.where(dqmax > 0.2)[0]
        settle_time = round(float(over[-1] * sim_cfg.dt), 2) if over.size else 0.0
        if cfg['ground']:
            settle_ok = bool(all(f > 5.0 for f in fn) and dqmax[-1] < 0.2
                             and np.isfinite(pitch))
        else:
            settle_ok = bool(dqmax[-1] < 0.2 and np.isfinite(pitch))
        out = dict(
            engine='physx', scenario=scen, init_condition=init_name,
            wheel_primitive=prim,
            hip_tau_Nm=[round(float(v), 2) for v in tau[0:4]],
            hip_q=[round(float(v), 3) for v in q[0:4]],
            pitch_deg=round(pitch, 2),
            Fn=[round(float(v), 1) for v in fn],
            n_contacts_per_wheel=[-1, -1, -1, -1],
            settle_ok=settle_ok, settle_time_s=settle_time,
            quat_xyzw=[round(float(x), 5) for x in quat],
            contact_pos_world={w: None for w in WHEELS},
            thigh_q=[round(float(v), 3) for v in q[4:8]],
            calf_q=[round(float(v), 3) for v in q[8:12]],
            thigh_tau=[round(float(v), 2) for v in tau[4:8]],
            calf_tau=[round(float(v), 2) for v in tau[8:12]],
            Fn_vec=[[round(float(x), 2) for x in v] for v in fn_vec],
            total_mass_kg=round(float(masses.sum()), 3),
            notes=('PhysX n_contacts=UNKNOWN (ContactSensor net force only); '
                   'base mass=delivery 10.08kg; friction 1.0; '
                   + ('cylinder collider (importer approximation UNKNOWN)'
                      if prim == 'cyl' else 'STL mesh collider')),
        )
        OUTD.mkdir(parents=True, exist_ok=True)
        (OUTD / f'{tag}.json').write_text(json.dumps(out, indent=1))
        print('RESULT ' + json.dumps(out), flush=True)
    except BaseException:
        import traceback
        traceback.print_exc()
        raise
    finally:
        try:
            app.close()
        except SystemExit:
            pass


def driver():
    LOGD.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env['OMNI_KIT_ACCEPT_EULA'] = 'YES'
    env['CUDA_VISIBLE_DEVICES'] = '1'
    env['LD_LIBRARY_PATH'] = '/home/robot/miniconda3/lib'
    for scen in ('S', 'G', 'GO'):
        for prim in ('mesh', 'cyl'):
            for init_name in ('real', 'nearflat', 'crouch'):
                tag = f'{scen}_{prim}_{init_name}'
                log = LOGD / f'{tag}.log'
                print('RUN', tag, flush=True)
                with open(log, 'w') as lf:
                    rc = subprocess.call(
                        [sys.executable, __file__, '--worker', '--scen', scen,
                         '--prim', prim, '--init', init_name],
                        env=env, stdout=lf, stderr=subprocess.STDOUT)
                print('  rc', rc, flush=True)
                if rc != 0:
                    print('  BLOCKED', tag, 'see', log, flush=True)
    merge()


def merge():
    rows = []
    # keep existing MuJoCo rows
    if TABLE.with_suffix('.json').exists():
        prev = json.loads(TABLE.with_suffix('.json').read_text())
        rows.extend([r for r in prev if r.get('engine') == 'mujoco'])
    for scen in ('S', 'G', 'GO'):
        for prim in ('mesh', 'cyl'):
            for init_name in ('real', 'nearflat', 'crouch'):
                f = OUTD / f'{scen}_{prim}_{init_name}.json'
                if f.exists():
                    rows.append(json.loads(f.read_text()))
                else:
                    rows.append(dict(
                        engine='physx', scenario=scen, init_condition=init_name,
                        wheel_primitive=prim, hip_tau_Nm=[], hip_q=[],
                        pitch_deg=float('nan'), Fn=[], n_contacts_per_wheel=[-1] * 4,
                        settle_ok=False, notes=f'BLOCKED: no result {f.name}'))
    fields = ['engine', 'scenario', 'init_condition', 'wheel_primitive',
              'hip_tau_Nm', 'hip_q', 'pitch_deg', 'Fn',
              'n_contacts_per_wheel', 'settle_ok', 'notes']
    with open(TABLE.with_suffix('.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({
                'engine': r['engine'], 'scenario': r['scenario'],
                'init_condition': r['init_condition'],
                'wheel_primitive': r['wheel_primitive'],
                'hip_tau_Nm': '/'.join(f'{x:.2f}' for x in r.get('hip_tau_Nm', [])),
                'hip_q': '/'.join(f'{x:.3f}' for x in r.get('hip_q', [])),
                'pitch_deg': r.get('pitch_deg'),
                'Fn': '/'.join(f'{x:.1f}' for x in r.get('Fn', [])),
                'n_contacts_per_wheel': '/'.join(f'{x:.2f}' for x in r.get('n_contacts_per_wheel', [])),
                'settle_ok': r.get('settle_ok'),
                'notes': r.get('notes', ''),
            })
    with open(TABLE.with_suffix('.json'), 'w') as f:
        json.dump(rows, f, indent=1)
    print('wrote combined matrix', len(rows), 'rows')


if __name__ == '__main__':
    P = argparse.ArgumentParser()
    P.add_argument('--worker', action='store_true')
    P.add_argument('--scen', choices=['S', 'G', 'GO'])
    P.add_argument('--prim', choices=['mesh', 'cyl'])
    P.add_argument('--init', choices=['real', 'nearflat', 'crouch'])
    a = P.parse_args()
    if a.worker:
        worker(a.scen, a.prim, a.init)
    else:
        driver()
