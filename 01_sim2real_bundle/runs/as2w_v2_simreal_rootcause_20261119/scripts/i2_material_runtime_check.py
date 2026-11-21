#!/usr/bin/env python3
"""I2: runtime verification of physics-material binding + corrected rerun.

G3's spawn-time bind_physics_material reportedly failed on instanced prims, so
the wheels ran at the default friction 1.0 instead of the requested value.

Step 1 (--check): spawn the robot with physics_material friction=0.8, do NOT
step, and read the runtime USD stage:
  * every Material prim under the robot and its friction attrs
  * every wheel collision prim's bound material and its static/dynamic friction
    (UsdPhysics.MaterialAPI.GetStaticFrictionAttr / GetDynamicFrictionAttr)
This is the closed-loop verification of the G3 defect.

Step 2 (driver): if wheel collision prims are found, create a dedicated
physics material (friction=0.8) and explicitly bind it to each wheel collision
prim BEFORE sim.reset() (so PhysX cooks with it), then rerun G real/nearflat.
Binding evidence chain is recorded verbatim in I2_MATERIAL_BINDING.json.
"""
import csv
import json
import pathlib
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import i_common as C  # noqa: E402

OUTD = C.ABL / 'i2_runs'
LOGD = C.ABL / 'i2_logs'
TABLE = C.TABLES / 'I2_MATERIAL_BINDING'
CHECK_JSON = OUTD / 'material_check.json'
FRIC = 0.8
DT = 0.005
NSTEP = 12000


def check():
    import isaaclab.sim as sim_utils
    from isaaclab.app import AppLauncher
    launcher = AppLauncher(headless=True, offscreen_render=False)
    app = launcher.app
    try:
        from isaaclab.actuators import ImplicitActuatorCfg
        from isaaclab.assets import ArticulationCfg
        from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
        from isaaclab.utils import configclass
        from isaaclab_physx.physics import PhysxCfg
        try:
            from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg as MatCfg
        except ImportError:
            MatCfg = sim_utils.RigidBodyMaterialCfg

        urdf = C.URDF['cyl']
        usd_cache = C.ABL / 'i_usd' / 'i2_check'

        @configclass
        class SceneCfg(InteractiveSceneCfg):
            robot = ArticulationCfg(
                prim_path='{ENV_REGEX_NS}/Robot',
                spawn=sim_utils.UrdfFileCfg(
                    asset_path=str(urdf), fix_base=False, merge_fixed_joints=True,
                    usd_dir=str(usd_cache), usd_file_name='as2w_i2_check.usd',
                    activate_contact_sensors=True, joint_drive=None,
                    physics_material=MatCfg(static_friction=FRIC,
                                            dynamic_friction=FRIC)),
                init_state=ArticulationCfg.InitialStateCfg(
                    pos=(0., 0., 0.55),
                    joint_pos={n: C.TGT_G[i] for i, n in enumerate(C.LEGS)}),
                actuators={'legs': ImplicitActuatorCfg(joint_names_expr=C.LEGS,
                                                       stiffness=55.0, damping=2.0)})

        sim_cfg = sim_utils.SimulationCfg(
            dt=DT, render_interval=4,
            physics=PhysxCfg(gpu_found_lost_aggregate_pairs_capacity=2**27),
            use_newton_actuators=False)
        sim = sim_utils.SimulationContext(sim_cfg)
        scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=3.))
        sim.reset()
        report = C.inspect_wheel_material(sim_utils)
        report['requested_friction'] = FRIC
        report['stage'] = 'after_reset_runtime'
        OUTD.mkdir(parents=True, exist_ok=True)
        CHECK_JSON.write_text(json.dumps(report, indent=1))
        print('CHECK ' + json.dumps(report)[:2000], flush=True)
    except BaseException:
        import traceback
        traceback.print_exc()
        raise
    finally:
        try:
            app.close()
        except SystemExit:
            pass


def build_runs():
    return [(f'fix_{init}', dict(prim='cyl', init=init, mode='implicit',
                                 dt=DT, nstep=NSTEP, kp=55.0, kd=2.0,
                                 friction=FRIC, bind_fix=True,
                                 outdir=str(OUTD)))
            for init in ('real', 'nearflat')]


def merge():
    check = json.loads(CHECK_JSON.read_text()) if CHECK_JSON.exists() else {}
    rows = []
    n_wheel = len(check.get('wheel_colliders', []))
    for init in ('real', 'nearflat'):
        f = OUTD / f'fix_{init}.json'
        if f.exists():
            r = json.loads(f.read_text())
            be = r.get('binding_evidence') or {}
            post = (be.get('post_reset') or {}).get('wheel_colliders', [])
            pre = (be.get('pre_bind') or {}).get('wheel_colliders', [])
            rows.append(dict(
                run=f'fix_{init}', init_condition=init,
                requested_friction=FRIC,
                pre_bind_material=[x.get('bound_material') for x in pre],
                pre_bind_friction=[x.get('friction') for x in pre],
                post_bind_material=[x.get('bound_material') for x in post],
                post_bind_friction=[x.get('friction') for x in post],
                n_bound=len(be.get('bound', [])),
                pitch_deg=r['pitch_deg'], hip_q=r['hip_q'],
                hip_tau_Nm=r['hip_tau_Nm'], Fn=r['Fn'],
                settle_ok=r['settle_ok'],
                binding_evidence=json.dumps(be)[:4000],
                notes='explicit runtime bind before sim.reset(); '
                      'Phase E/f3b: static torque insensitive to friction (<=0.3 Nm)',
            ))
        else:
            rows.append(dict(run=f'fix_{init}', init_condition=init,
                             requested_friction=FRIC,
                             binding_evidence='',
                             notes=f'BLOCKED: no result {f.name}'))
    # verification report row
    rows.append(dict(
        run='RUNTIME_CHECK', init_condition='-', requested_friction=FRIC,
        pre_bind_material=[m['path'] for m in check.get('materials', [])],
        pre_bind_friction=[m['friction_attrs'] for m in check.get('materials', [])],
        post_bind_material=[x.get('bound_material')
                            for x in check.get('wheel_colliders', [])],
        post_bind_friction=[x.get('friction')
                            for x in check.get('wheel_colliders', [])],
        n_bound=n_wheel,
        binding_evidence=json.dumps(check)[:4000],
        notes=('spawn-time material check at runtime stage (no step). '
               'If bound_material is None or friction != requested -> G3 defect '
               'reproduced (default friction 1.0).'),
    ))
    fields = ['run', 'init_condition', 'requested_friction', 'pre_bind_material',
              'pre_bind_friction', 'post_bind_material', 'post_bind_friction',
              'n_bound', 'pitch_deg', 'hip_q', 'hip_tau_Nm', 'Fn', 'settle_ok',
              'binding_evidence', 'notes']
    with open(TABLE.with_suffix('.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({**r,
                        'hip_q': '/'.join(f'{x:.4f}' for x in r.get('hip_q', [])),
                        'hip_tau_Nm': '/'.join(f'{x:.2f}' for x in r.get('hip_tau_Nm', [])),
                        'Fn': '/'.join(f'{x:.1f}' for x in r.get('Fn', []))})
    with open(TABLE.with_suffix('.json'), 'w') as f:
        json.dump(dict(runtime_check=check, rows=rows), f, indent=1)
    print('wrote', TABLE)


def main():
    OUTD.mkdir(parents=True, exist_ok=True)
    LOGD.mkdir(parents=True, exist_ok=True)
    env = __import__('os').environ.copy()
    env['OMNI_KIT_ACCEPT_EULA'] = 'YES'
    env['CUDA_VISIBLE_DEVICES'] = '1'
    env['LD_LIBRARY_PATH'] = '/home/robot/miniconda3/lib'
    log = LOGD / 'material_check.log'
    print('RUN material_check', flush=True)
    with open(log, 'w') as lf:
        subprocess.call([sys.executable, str(pathlib.Path(__file__).resolve()),
                         '--check'], env=env, stdout=lf,
                        stderr=subprocess.STDOUT)
    check_ok = CHECK_JSON.exists() and json.loads(
        CHECK_JSON.read_text()).get('wheel_colliders')
    if not check_ok:
        print('CHECK found no wheel colliders -> BLOCKED, no rerun', flush=True)
        rows = [dict(run='RUNTIME_CHECK', init_condition='-',
                     requested_friction=FRIC,
                     binding_evidence=json.dumps(
                         json.loads(CHECK_JSON.read_text())
                         if CHECK_JSON.exists() else {}),
                     notes='BLOCKED: no wheel collision prims found')]
        with open(TABLE.with_suffix('.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=['run', 'init_condition',
                                              'requested_friction',
                                              'binding_evidence', 'notes'],
                               extrasaction='ignore')
            w.writeheader()
            for r in rows:
                w.writerow(r)
        with open(TABLE.with_suffix('.json'), 'w') as f:
            json.dump(dict(rows=rows), f, indent=1)
        return
    C.drive(pathlib.Path(__file__).resolve(), build_runs(), LOGD)
    merge()


if __name__ == '__main__':
    if '--worker' in sys.argv:
        C.worker_cli()
    elif '--check' in sys.argv:
        check()
    else:
        main()
