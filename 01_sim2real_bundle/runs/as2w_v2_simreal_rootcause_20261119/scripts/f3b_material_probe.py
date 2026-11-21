#!/usr/bin/env python3
"""Phase E audit probe: verify that UrdfFileCfg(physics_material=...) actually
authors friction onto the spawned collision prims (runtime stage, not the cached
USD). Spawns the C_fric0.1 config, prints every physics material binding found
under the robot prim, then exits without stepping.

Run: OMNI_KIT_ACCEPT_EULA=YES CUDA_VISIBLE_DEVICES=1 \
     /home/robot/z00945176/isaac_env/venv312/bin/python f3b_material_probe.py
"""
import pathlib

REPO = pathlib.Path('/home/robot/z00945176/Rob2Nav')
RUNS = REPO / 'runs/as2w_v2_simreal_rootcause_20261119'
URDF = pathlib.Path('/tmp/as2w-unitreestyle/calibration/candidates/load-v0/as2w-load-v0.urdf')

from isaaclab.app import AppLauncher  # noqa: E402

launcher = AppLauncher(headless=True, offscreen_render=False)
app = launcher.app

try:
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.utils import configclass
    from isaaclab_physx.physics import PhysxCfg
    from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg as MatCfg
    from pxr import Usd, UsdShade, UsdPhysics

    LEGS = [f'{leg}_{part}' for part in ('hip', 'thigh', 'calf')
            for leg in ('FL', 'FR', 'RL', 'RR')]
    WHEELS = [f'{leg}_foot' for leg in ('FL', 'FR', 'RL', 'RR')]

    sim_cfg = sim_utils.SimulationCfg(
        dt=0.005, render_interval=4,
        physics=PhysxCfg(gpu_found_lost_aggregate_pairs_capacity=2**27),
        use_newton_actuators=False)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        robot = ArticulationCfg(
            prim_path='{ENV_REGEX_NS}/Robot',
            spawn=sim_utils.UrdfFileCfg(
                asset_path=str(URDF), fix_base=False, merge_fixed_joints=True,
                usd_dir=str(RUNS / 'physx_abl' / 'usd_C_fric0.1'),
                usd_file_name='as2w_C_fric0.1.usd',
                activate_contact_sensors=True, joint_drive=None,
                physics_material=MatCfg(static_friction=0.1, dynamic_friction=0.1)),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0., 0., 0.55),
                joint_pos={n: v for n, v in zip(LEGS, [0, 0, 0, 0, 0.8, 0.8, 0.8, 0.8,
                                                       -1.6, -1.6, -1.6, -1.6])}),
            actuators={'legs': ImplicitActuatorCfg(joint_names_expr=LEGS,
                                                   stiffness=55.0, damping=2.0)})

    sim = sim_utils.SimulationContext(sim_cfg)
    scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=3.))
    sim.reset()

    stage = sim_utils.get_current_stage()
    root = '/World/envs/env_0/Robot'
    print('=== material prims under robot ===', flush=True)
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if not p.startswith(root):
            continue
        if prim.GetTypeName() == 'Material':
            attrs = {a.GetName(): a.Get() for a in prim.GetAttributes()
                     if 'riction' in a.GetName() or 'ompliant' in a.GetName()}
            print('MAT', p, attrs, flush=True)
    print('=== collision prim bindings (first 8) ===', flush=True)
    n = 0
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if not p.startswith(root):
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            mat, rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            print('COLL', p, '->', mat.GetPath() if mat else None, flush=True)
            n += 1
            if n >= 8:
                break
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    try:
        app.close()
    except SystemExit:
        pass
