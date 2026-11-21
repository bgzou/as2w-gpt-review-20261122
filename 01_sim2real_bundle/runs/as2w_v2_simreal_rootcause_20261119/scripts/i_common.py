#!/usr/bin/env python3
"""Shared PhysX-side Phase-I harness helpers (I1/I3/I4).

One app process per run, driven by a parent driver exactly like g3_physx_matrix.py.
Keeps the two G3 pitfalls fixed:
  * InitialStateCfg.joint_pos is silently ignored -> write_joint_position_to_sim_index
    after sim.reset().
  * bind_physics_material on instanced prims may fail -> friction is reported in the
    result notes and validated separately by i2_material_runtime_check.py.

Modes:
  explicit  : zero all drive stiffness/damping, compute tau = kp*(tgt-q) + kd*(0-dq)
              every sim step and inject via set_joint_effort_target_index
              (PhysX set_dof_actuation_forces). Wheels tau = clip(-2*dq, -15, 15).
  implicit  : G3 behaviour (ImplicitActuatorCfg stiffness/damping, position target).
"""
import json
import os
import pathlib
import subprocess
import sys

import numpy as np

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'
ABL = RUNS / 'physx_abl'
NPZ = RUNS / 'qc' / 'a1_timelines' / 'P55G2.npz'
TABLES = RUNS / 'tables'

LEGS = [f'{leg}_{part}' for part in ('hip', 'thigh', 'calf')
        for leg in ('FL', 'FR', 'RL', 'RR')]
WHEELS = [f'{leg}_foot' for leg in ('FL', 'FR', 'RL', 'RR')]
R_WHEEL = 0.086
GRAV = 9.81
DELIVERY_MASS = 26.0            # model mass (base 10.08 + legs/wheels), NOT weighed
WEIGHT_N = DELIVERY_MASS * GRAV  # 255.06 N, model convention

TGT_G = [0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8, -1.6, -1.6, -1.6, -1.6]
# explicit torque limits per leg joint order (part-major): hip 60, thigh 60, calf 90
TAU_LIM = [60.0] * 4 + [60.0] * 4 + [90.0] * 4
WHEEL_TAU_LIM = 15.0
WHEEL_DAMP = 2.0

URDF = {'mesh': ABL / 'urdf_meshwheel' / 'as2w-load-v0.urdf',
        'cyl': ABL / 'urdf_cylwheel' / 'as2w-load-v0.urdf'}


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


def _pitch_to_quat_xyzw(deg):
    th = np.radians(deg)
    # rotation about +y (pitch)
    return [0.0, float(np.sin(th / 2)), 0.0, float(np.cos(th / 2))]


def inspect_wheel_material(sim_utils):
    """Read runtime-stage physics material state for wheel collision prims."""
    from pxr import UsdPhysics, UsdShade
    stage = sim_utils.get_current_stage()
    out = {'materials': [], 'wheel_colliders': [], 'n_colliders': 0}
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if '/Robot' not in p:
            continue
        if prim.GetTypeName() == 'Material':
            attrs = {}
            for a in prim.GetAttributes():
                if 'riction' in a.GetName():
                    attrs[a.GetName()] = a.Get()
            out['materials'].append({'path': p, 'friction_attrs': attrs})
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            out['n_colliders'] += 1
            if 'foot' in p.lower():
                mba = UsdShade.MaterialBindingAPI(prim)
                mat_phys, _ = mba.ComputeBoundMaterial(materialPurpose='physics')
                mat_any, _ = mba.ComputeBoundMaterial()
                mat = mat_phys if mat_phys else mat_any
                mpath = str(mat.GetPath()) if mat else None
                fr = None
                if mat:
                    mp = stage.GetPrimAtPath(mpath)
                    if mp.HasAPI(UsdPhysics.MaterialAPI):
                        api = UsdPhysics.MaterialAPI(mp)
                        fr = {'static': api.GetStaticFrictionAttr().Get(),
                              'dynamic': api.GetDynamicFrictionAttr().Get()}
                out['wheel_colliders'].append(
                    {'path': p, 'bound_material_physics': str(mat_phys.GetPath())
                     if mat_phys else None,
                     'bound_material_anypurpose': str(mat_any.GetPath())
                     if mat_any else None,
                     'bound_material': mpath, 'friction': fr})
    return out


def bind_wheel_material(sim_utils, friction):
    """Create a physics material and bind it to every wheel collision prim.

    Uses UsdShade.MaterialBindingAPI directly with materialPurpose='physics'
    (sim_utils.bind_physics_material returned False in this build).
    """
    from pxr import UsdPhysics, UsdShade
    stage = sim_utils.get_current_stage()
    mat_path = '/World/PhysicsMaterials/wheel_fric'
    mp = stage.GetPrimAtPath(mat_path)
    if not mp.IsValid():
        mp = stage.DefinePrim(mat_path, 'Material')
    api = UsdPhysics.MaterialAPI.Apply(mp)
    api.CreateStaticFrictionAttr(float(friction))
    api.CreateDynamicFrictionAttr(float(friction))
    material = UsdShade.Material(mp)
    bound = []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if '/Robot' in p and 'foot' in p.lower() and prim.HasAPI(UsdPhysics.CollisionAPI):
            mba = UsdShade.MaterialBindingAPI.Apply(prim)
            mba.Bind(material,
                     bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                     materialPurpose='physics')
            chk, _ = mba.ComputeBoundMaterial(materialPurpose='physics')
            bound.append({'path': p, 'bound': bool(chk)})
    print('BIND_FIX', len(bound),
          [(b['path'].split('/')[-2], b['bound']) for b in bound], flush=True)
    return {'material_path': mat_path, 'friction': friction,
            'bound': bound, 'pre_bind': inspect_wheel_material(sim_utils)}


def worker(params):
    """Run one settle. params keys:
    tag, prim, init, mode(explicit|implicit), dt, nstep, kp, kd,
    pitch_deg, hip_offset, gap_mm, friction, record_contacts, outdir
    """
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

        prim = params['prim']
        init_name = params['init']
        mode = params.get('mode', 'implicit')
        dt = float(params.get('dt', 0.005))
        nstep = int(params.get('nstep', 12000))
        KP = float(params.get('kp', 55.0))
        KD = float(params.get('kd', 2.0))
        pitch0 = float(params.get('pitch_deg', 0.0))
        hip_off = float(params.get('hip_offset', 0.0))
        gap_mm = float(params.get('gap_mm', 2.0))
        friction = float(params.get('friction', 1.0))
        record_contacts = bool(params.get('record_contacts', False))
        outdir = pathlib.Path(params['outdir'])
        tag = params['tag']

        q0 = list(INITS[init_name])
        # hip joints are indices 0..3 (part-major)
        q0 = [q0[i] + (hip_off if i < 4 else 0.0) for i in range(12)]
        urdf = URDF[prim]
        limits = {j.get('name'): j.find('limit').attrib
                  for j in ET.parse(urdf).findall('joint')
                  if j.find('limit') is not None}
        usd_cache = ABL / 'i_usd' / f'{prim}_{init_name}_{tag}'

        sim_cfg = sim_utils.SimulationCfg(
            dt=dt, render_interval=4,
            physics=PhysxCfg(gpu_found_lost_aggregate_pairs_capacity=2**27),
            use_newton_actuators=False)

        @configclass
        class SceneCfg(InteractiveSceneCfg):
            robot = ArticulationCfg(
                prim_path='{ENV_REGEX_NS}/Robot',
                spawn=sim_utils.UrdfFileCfg(
                    asset_path=str(urdf), fix_base=False,
                    merge_fixed_joints=True, usd_dir=str(usd_cache),
                    usd_file_name=f'as2w_{tag}.usd',
                    activate_contact_sensors=True, joint_drive=None,
                    physics_material=MatCfg(static_friction=friction,
                                            dynamic_friction=friction)),
                init_state=ArticulationCfg.InitialStateCfg(
                    pos=(0., 0., 0.55),
                    joint_pos={n: q0[i] for i, n in enumerate(LEGS)}),
                actuators={
                    'legs': ImplicitActuatorCfg(
                        joint_names_expr=LEGS, stiffness=KP if mode == 'implicit' else 0.0,
                        damping=KD if mode == 'implicit' else 0.0,
                        effort_limit_sim={j: float(limits[j]['effort']) for j in LEGS},
                        velocity_limit_sim={j: float(limits[j]['velocity']) for j in LEGS}),
                    'wheels': ImplicitActuatorCfg(
                        joint_names_expr=WHEELS, stiffness=0.0,
                        damping=WHEEL_DAMP if mode == 'implicit' else 0.0,
                        effort_limit_sim={j: float(limits[j]['effort']) for j in WHEELS},
                        velocity_limit_sim={j: float(limits[j]['velocity']) for j in WHEELS}),
                })
            contact_forces = ContactSensorCfg(
                prim_path='{ENV_REGEX_NS}/Robot/.*', history_length=1,
                track_contact_points=record_contacts,
                track_friction_forces=record_contacts,
                filter_prim_paths_expr=['/World/ground'] if record_contacts else [],
                max_contact_data_count_per_prim=64 if record_contacts else None)

        sim = sim_utils.SimulationContext(sim_cfg)
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
        bind_evidence = None
        sim.reset()
        robot.update(dt)
        if params.get('bind_fix'):
            # collision prims only carry CollisionAPI after the first reset;
            # bind then reset again so PhysX re-cooks shapes with the material.
            bind_evidence = bind_wheel_material(sim_utils, friction)
            sim.reset()
            robot.update(dt)
            bind_evidence['post_reset'] = inspect_wheel_material(sim_utils)

        names = robot.joint_names
        leg_ids = [names.index(n) for n in LEGS]
        wheel_ids = [names.index(n) for n in WHEELS]
        all_ids = leg_ids + wheel_ids
        body_names = robot.body_names
        masses = np.asarray(robot.root_physx_view.get_masses()).reshape(-1)
        print('TOTAL_MASS', round(float(masses.sum()), 3), flush=True)

        dev = robot.device
        # explicit joint state (InitialStateCfg.joint_pos is ignored in this build)
        q0_t = torch.tensor([[q0[i] for i in range(12)]], device=dev)
        robot.write_joint_position_to_sim_index(position=q0_t, joint_ids=leg_ids)
        robot.write_joint_velocity_to_sim_index(
            velocity=torch.zeros_like(q0_t), joint_ids=leg_ids)
        robot.write_joint_position_to_sim_index(
            position=torch.zeros(1, len(WHEELS), device=dev), joint_ids=wheel_ids)
        robot.update(dt)

        # root orientation (pitch offset) then wheel-lowest-point gap placement
        rp = robot.data.root_pose_w.clone()  # (x,y,z,qx,qy,qz,qw)
        rp[0, 3:7] = torch.tensor(_pitch_to_quat_xyzw(pitch0), device=dev)
        robot.write_root_pose_to_sim_index(root_pose=rp)
        robot.update(dt)
        bw = robot.data.body_pos_w[0].cpu().numpy()
        wz = [float(bw[body_names.index(w)][2]) for w in WHEELS]
        lowest = min(wz) - R_WHEEL
        rp = robot.data.root_pose_w.clone()
        rp[0, 2] = rp[0, 2] + (gap_mm / 1000.0 - lowest)
        robot.write_root_pose_to_sim_index(root_pose=rp)
        robot.update(dt)
        print('INIT_Q', [round(float(x), 4) for x in
                         robot.data.joint_pos[0, leg_ids].cpu().numpy()], flush=True)
        print('START_Z', round(float(rp[0, 2].item()), 4),
              'pitch0', pitch0, 'gap_mm', gap_mm, flush=True)

        tgt_leg = torch.tensor([[TGT_G[i] for i in range(12)]], device=dev)
        tau_lim = torch.tensor([TAU_LIM], device=dev)
        rec_from = int(nstep * 0.8)
        hist = {'tau': [], 'q': [], 'quat': [], 'fn': [], 'fn_vec': [],
                'dqmax': [], 'dq_leg': []}
        if record_contacts:
            hist.update({'fn_normal': [], 'fn_friction': [], 'contact_pos': [],
                         'root_com': [], 'root_pose': []})

        if mode == 'implicit':
            robot.write_joint_stiffness_to_sim_index(stiffness=KP, joint_ids=leg_ids)
            robot.write_joint_damping_to_sim_index(damping=KD, joint_ids=leg_ids)
            robot.write_joint_stiffness_to_sim_index(stiffness=0.0, joint_ids=wheel_ids)
            robot.write_joint_damping_to_sim_index(damping=WHEEL_DAMP, joint_ids=wheel_ids)
        else:
            # zero all drives so only explicit effort acts
            robot.write_joint_stiffness_to_sim_index(stiffness=0.0, joint_ids=all_ids)
            robot.write_joint_damping_to_sim_index(damping=0.0, joint_ids=all_ids)

        sensor = scene['contact_forces']
        s_names = list(sensor.body_names)
        wheel_sids = [s_names.index(w) for w in WHEELS]

        for step in range(nstep):
            if mode == 'implicit':
                robot.set_joint_position_target(tgt_leg, joint_ids=leg_ids)
                robot.set_joint_velocity_target(
                    torch.zeros(1, len(WHEELS), device=dev), joint_ids=wheel_ids)
                scene.write_data_to_sim()
                sim.step()
                scene.update(dt)
                tau_now = robot.data.applied_torque[:, all_ids][0].cpu().numpy().copy()
            else:
                q_leg = robot.data.joint_pos[0, leg_ids]
                dq_leg = robot.data.joint_vel[0, leg_ids]
                q_w = robot.data.joint_pos[0, wheel_ids]
                dq_w = robot.data.joint_vel[0, wheel_ids]
                tau_leg = KP * (tgt_leg[0] - q_leg) + KD * (0.0 - dq_leg)
                tau_leg = torch.clamp(tau_leg, -tau_lim[0], tau_lim[0])
                tau_w = torch.clamp(-WHEEL_DAMP * dq_w, -WHEEL_TAU_LIM, WHEEL_TAU_LIM)
                tau_cmd = torch.cat([tau_leg, tau_w]).unsqueeze(0)
                robot.set_joint_effort_target_index(target=tau_cmd, joint_ids=all_ids)
                scene.write_data_to_sim()
                sim.step()
                scene.update(dt)
                tau_now = tau_cmd[0].cpu().numpy().copy()
            dqv = robot.data.joint_vel[0, leg_ids]
            hist['dqmax'].append(float(dqv.abs().max().item()))
            if step >= rec_from:
                hist['dq_leg'].append(robot.data.joint_vel[0, leg_ids].cpu().numpy().copy())
                q = robot.data.joint_pos[:, all_ids]
                hist['tau'].append(tau_now)
                hist['q'].append(q[0].cpu().numpy().copy())
                hist['quat'].append(robot.data.root_quat_w[0].cpu().numpy().copy())
                fnv = sensor.data.net_forces_w[0, wheel_sids]
                hist['fn'].append(fnv.norm(dim=-1).cpu().numpy().copy())
                hist['fn_vec'].append(fnv.cpu().numpy().copy())
                if record_contacts:
                    try:
                        nn = sensor.data.net_normal_forces_w[0, wheel_sids]
                        hist['fn_normal'].append(nn.cpu().numpy().copy())
                    except Exception:
                        hist['fn_normal'].append(np.full((4, 3), np.nan))
                    try:
                        ff = sensor.data.net_friction_forces_w[0, wheel_sids]
                        hist['fn_friction'].append(ff.cpu().numpy().copy())
                    except Exception:
                        hist['fn_friction'].append(np.full((4, 3), np.nan))
                    try:
                        cp = sensor.data.contact_pos_w[0, wheel_sids]
                        hist['contact_pos'].append(cp.cpu().numpy().copy())
                    except Exception:
                        hist['contact_pos'].append(np.full((4, 1, 3), np.nan))
                    hist['root_com'].append(
                        robot.data.root_com_pose_w[0, :3].cpu().numpy().copy())
                    hist['root_pose'].append(
                        robot.data.root_pose_w[0].cpu().numpy().copy())

        tau = np.stack(hist['tau']).mean(0)
        q = np.stack(hist['q']).mean(0)
        fn = np.stack(hist['fn']).mean(0)
        fn_vec = np.stack(hist['fn_vec']).mean(0)
        quat = np.stack(hist['quat']).mean(0)  # (x,y,z,w)
        pitch = float(np.degrees(np.arcsin(np.clip(
            -2 * (quat[1] * quat[3] - quat[0] * quat[2]), -1, 1))))
        dqmax = np.array(hist['dqmax'])
        over = np.where(dqmax > 0.2)[0]
        settle_time = round(float(over[-1] * dt), 2) if over.size else 0.0
        settle_ok = bool(all(f > 5.0 for f in fn) and dqmax[-1] < 0.2
                         and np.isfinite(pitch))
        out = dict(
            engine='physx', scenario='G', init_condition=init_name,
            wheel_primitive=prim, mode=mode, dt=dt, nstep=nstep,
            kp=KP, kd=KD, pitch_init_deg=pitch0, hip_offset_rad=hip_off,
            gap_mm=gap_mm, friction=friction,
            hip_tau_Nm=[round(float(v), 2) for v in tau[0:4]],
            hip_q=[round(float(v), 4) for v in q[0:4]],
            pitch_deg=round(pitch, 3),
            Fn=[round(float(v), 2) for v in fn],
            n_contacts_per_wheel=[-1, -1, -1, -1],
            settle_ok=settle_ok, settle_time_s=settle_time,
            dqmax_last=round(float(dqmax[-1]), 4),
            dqmax_p99=round(float(np.percentile(dqmax, 99)), 4),
            dqmax_settled_mean=round(float(dqmax[rec_from:].mean()), 4),
            dq_leg_settled_max=[round(float(x), 3) for x in
                                np.abs(np.stack(hist['dq_leg'])).max(0)] if 'dq_leg' in hist else None,
            quat_xyzw=[round(float(x), 5) for x in quat],
            thigh_q=[round(float(v), 4) for v in q[4:8]],
            calf_q=[round(float(v), 4) for v in q[8:12]],
            thigh_tau=[round(float(v), 2) for v in tau[4:8]],
            calf_tau=[round(float(v), 2) for v in tau[8:12]],
            Fn_vec=[[round(float(x), 2) for x in v] for v in fn_vec],
            total_mass_kg=round(float(masses.sum()), 3),
            notes=('PhysX n_contacts=UNKNOWN (ContactSensor net force only); '
                   f'mode={mode}; base mass=delivery 10.08kg; friction={friction}; '
                   + ('cylinder collider' if prim == 'cyl' else 'STL mesh collider')),
        )
        if bind_evidence is not None:
            out['binding_evidence'] = bind_evidence
        if record_contacts:
            out['contact_series'] = {
                'Fn_vec': np.stack(hist['fn_vec']).round(3).tolist(),
                'Fn_normal': np.stack(hist['fn_normal']).round(3).tolist(),
                'Fn_friction': np.stack(hist['fn_friction']).round(3).tolist(),
                'contact_pos': np.stack(hist['contact_pos']).round(4).tolist(),
                'root_com': np.stack(hist['root_com']).round(4).tolist(),
                'root_pose': np.stack(hist['root_pose']).round(5).tolist(),
                'wheel_names': WHEELS,
            }
        outdir.mkdir(parents=True, exist_ok=True)
        (outdir / f'{tag}.json').write_text(json.dumps(out, indent=1))
        print('RESULT ' + json.dumps({k: v for k, v in out.items()
                                      if k != 'contact_series'}), flush=True)
    except BaseException:
        import traceback
        traceback.print_exc()
        raise
    finally:
        try:
            app.close()
        except SystemExit:
            pass


def drive(script, runs, logdir):
    """runs: list of (tag, params dict). Spawns one app process per run."""
    logdir = pathlib.Path(logdir)
    logdir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env['OMNI_KIT_ACCEPT_EULA'] = 'YES'
    env['CUDA_VISIBLE_DEVICES'] = '1'
    env['LD_LIBRARY_PATH'] = '/home/robot/miniconda3/lib'
    for tag, p in runs:
        p = dict(p)
        p['tag'] = tag
        log = logdir / f'{tag}.log'
        print('RUN', tag, flush=True)
        with open(log, 'w') as lf:
            rc = subprocess.call(
                [sys.executable, str(script), '--worker',
                 '--params', json.dumps(p)],
                env=env, stdout=lf, stderr=subprocess.STDOUT)
        print('  rc', rc, flush=True)
        if rc != 0:
            print('  BLOCKED', tag, 'see', log, flush=True)


def worker_cli():
    import argparse
    P = argparse.ArgumentParser()
    P.add_argument('--worker', action='store_true')
    P.add_argument('--params', type=str)
    a = P.parse_args()
    if not a.worker:
        raise SystemExit('use driver mode in the per-task script')
    worker(json.loads(a.params))
